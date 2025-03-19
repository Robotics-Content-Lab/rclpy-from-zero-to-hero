import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from ch5_interfaces.srv import Takeoff, Land
from ch5_interfaces.action import DriveCircle
import math
from concurrent.futures import ThreadPoolExecutor

class DroneMissionNodeServer(Node):
    def __init__(self):
        super().__init__('drone_mission_node_server')
        
        # Thread-safe state management
        self._state_lock = threading.Lock()
        self._airborne = False
        self._current_altitude = 0.0
        self._current_operation = 'IDLE'  # IDLE|TAKEOFF|LANDING|CIRCLE
        self._altitude_updated = threading.Event()
        self._circle_complete = threading.Event()
        
        # Thread pool for circle execution
        self._executor = ThreadPoolExecutor(max_workers=1)

        # Callback groups for concurrent operations
        self._service_group = ReentrantCallbackGroup()
        self._action_group = ReentrantCallbackGroup()
        self._monitor_group = ReentrantCallbackGroup()

        # Publishers
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/simple_drone/land', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)

        # Subscribers
        self.create_subscription(
            Range,
            '/simple_drone/sonar/out',
            self.sonar_callback,
            10,
            callback_group=self._monitor_group
        )

        self.ensure_publisher_connected()

        # Service servers
        self.takeoff_service = self.create_service(
            Takeoff,
            'Takeoff',
            self.handle_takeoff,
            callback_group=self._service_group
        )
        self.land_service = self.create_service(
            Land,
            'Land',
            self.handle_land,
            callback_group=self._service_group
        )

        # Action server
        self._action_server = ActionServer(
            self,
            DriveCircle,
            'DriveCircle',
            execute_callback=self.execute_drive_circle_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group
        )

        self.get_logger().info("Drone Mission Node Server started")

    def ensure_publisher_connected(self):
        while self.cmd_vel_pub.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for subscriber on {self.cmd_vel_pub.topic} ... ",
                                   throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)

    def sonar_callback(self, msg: Range):
        with self._state_lock:
            self._current_altitude = msg.range
            # Set the event to signal an altitude update.
            self._altitude_updated.set()

    def _monitor_altitude(self, target, timeout, check_fn):
        """
        Monitor altitude with timeout using threading.Event.
        The event is cleared before checking altitude to ensure no updates are missed.
        """
        end_time = time.time() + timeout
        while time.time() < end_time:
            # Clear the event before checking to avoid missing updates
            self._altitude_updated.clear()
            
            with self._state_lock:
                current_alt = self._current_altitude
            if check_fn(current_alt):
                return True

            remaining_time = end_time - time.time()
            if remaining_time <= 0:
                break
            # Wait for a new update (or timeout)
            self._altitude_updated.wait(timeout=min(0.1, remaining_time))
        return False

    def handle_takeoff(self, request, response):
        """Handle takeoff service request synchronously"""
        with self._state_lock:
            if self._current_operation != 'IDLE':
                response.success = False
                return response
            self._current_operation = 'TAKEOFF'

        try:
            self.get_logger().info(f"Takeoff requested to altitude: {request.altitude}")
            self.takeoff_pub.publish(Empty())

            success = self._monitor_altitude(
                target=request.altitude,
                timeout=5.0,
                check_fn=lambda x: x >= request.altitude
            )

            with self._state_lock:
                if success:
                    self._airborne = True
                    self.get_logger().info("Takeoff successful")
                else:
                    self.get_logger().error("Takeoff failed - timeout reached")
                    self.get_logger().error(f"Altitude: {self._current_altitude}")
                response.success = success
                self._current_operation = 'IDLE'
            return response

        except Exception as e:
            self.get_logger().error(f"Takeoff error: {str(e)}")
            self.emergency_land()
            response.success = False
            return response

    def handle_land(self, request, response):
        """Handle landing service request synchronously"""
        with self._state_lock:
            if self._current_operation != 'IDLE':
                response.success = False
                return response
            self._current_operation = 'LANDING'

        try:
            self.get_logger().info("Land requested")
            self.land_pub.publish(Empty())

            success = self._monitor_altitude(
                target=0.0,
                timeout=5.0,
                check_fn=lambda x: x <= 0.1  # Small threshold for landing
            )

            with self._state_lock:
                if success:
                    self._airborne = False
                    self.get_logger().info("Landing successful")
                else:
                    self.get_logger().error("Landing failed - timeout reached")
                response.success = success
                self._current_operation = 'IDLE'
            return response

        except Exception as e:
            self.get_logger().error(f"Landing error: {str(e)}")
            response.success = False
            return response

    def emergency_land(self):
        """Emergency landing procedure (synchronous)"""
        self.get_logger().warn("Initiating emergency landing")
        with self._state_lock:
            self._current_operation = 'LANDING'

        self.cmd_vel_pub.publish(Twist())  # Stop all motion
        self.land_pub.publish(Empty())

        success = self._monitor_altitude(
            target=0.0,
            timeout=5.0,
            check_fn=lambda x: x <= 0.1
        )

        with self._state_lock:
            self._airborne = False
            self._current_operation = 'IDLE'
        return success

    def goal_callback(self, goal_request):
        """Validate action goals"""
        with self._state_lock:
            if not self._airborne or self._current_operation != 'IDLE':
                self.get_logger().warn("Cannot accept goal - drone not ready")
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle action cancellation"""
        self.get_logger().info("DriveCircle goal canceled")
        self._circle_complete.set()  # Signal circle completion to stop waiting
        return CancelResponse.ACCEPT

    def _execute_circle(self, linear_vel, angular_vel, duration):
        """Execute one circle iteration"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)
        self._circle_complete.wait(timeout=duration)
        return not self._circle_complete.is_set()  # Return True if not canceled

    async def execute_drive_circle_callback(self, goal_handle):
        """Execute the DriveCircle action"""
        with self._state_lock:
            if not self._airborne or self._current_operation != 'IDLE':
                return DriveCircle.Result(success=False)
            self._current_operation = 'CIRCLE'

        try:
            self.get_logger().info("Executing DriveCircle action")
            radius = goal_handle.request.radius
            repetitions = goal_handle.request.repetitions
            time_limit = goal_handle.request.time_limit

            result = DriveCircle.Result()
            result.success = False
            feedback_msg = DriveCircle.Feedback()
            start_time = self.get_clock().now()
            linear_vel = 0.5  # m/s

            # Calculate angular velocity and circle duration
            angular_vel = linear_vel / radius if radius > 0 else 0.0
            circle_duration = 2 * math.pi * radius / linear_vel

            current_iteration = 0
            self._circle_complete.clear()

            while current_iteration < repetitions:
                # Check for timeout
                if (self.get_clock().now() - start_time).nanoseconds / 1e9 > time_limit:
                    self.get_logger().error("DriveCircle timed out")
                    self.emergency_land()
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().info("DriveCircle canceled")
                    self.emergency_land()
                    return result

                # Execute one circle iteration in the thread pool
                future = self._executor.submit(
                    self._execute_circle,
                    linear_vel,
                    angular_vel,
                    circle_duration
                )

                if not future.result():  # Circle was canceled
                    self.emergency_land()
                    return result

                # Update feedback
                current_iteration += 1
                feedback_msg.current_iteration = current_iteration
                goal_handle.publish_feedback(feedback_msg)

            # Stop motion
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("DriveCircle completed successfully")

            with self._state_lock:
                self._current_operation = 'IDLE'

            goal_handle.succeed()
            result.success = True
            return result

        except Exception as e:
            self.get_logger().error(f"DriveCircle error: {str(e)}")
            self.emergency_land()
            return result
        finally:
            with self._state_lock:
                self._current_operation = 'IDLE'

    def destroy_node(self):
        self._executor.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DroneMissionNodeServer()
        # Use MultiThreadedExecutor with 4 threads
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()