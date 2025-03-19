#!/usr/bin/env python3
import rclpy
import threading
from concurrent.futures import ThreadPoolExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from ch5_interfaces.srv import Takeoff, Land
from ch5_interfaces.action import DriveCircle

class DroneMissionNode(Node):
    def __init__(self):
        super().__init__('drone_mission_node')
        
        # Create callback group for concurrent service/action calls
        self._cb_group = ReentrantCallbackGroup()
        
        # Service clients
        self._takeoff_client = self.create_client(
            Takeoff, 
            'Takeoff',
            callback_group=self._cb_group
        )
        self._land_client = self.create_client(
            Land,
            'Land',
            callback_group=self._cb_group
        )
        
        # Action client
        self._drive_circle_client = ActionClient(
            self,
            DriveCircle,
            'DriveCircle',
            callback_group=self._cb_group
        )
        
        # Mission parameters
        self._target_altitude = 0.80  # meters
        self._circle_radius = 1.0  # meters
        self._circle_repetitions = 2
        self._time_limit = 60  # seconds
        
        # Threading events
        self._shutdown = threading.Event()
        self._takeoff_complete = threading.Event()
        self._circle_complete = threading.Event()
        self._landing_complete = threading.Event()
        
        # Thread pool for concurrent tasks
        self._executor = ThreadPoolExecutor(max_workers=3)
        
        self.get_logger().info('Drone mission node initialized')

    def start_ros_spin(self):
        """Start ROS executor spinning in a separate thread"""
        def run_executor():
            executor = MultiThreadedExecutor()
            executor.add_node(self)
            try:
                while not self._shutdown.is_set():
                    executor.spin_once(timeout_sec=0.1)
            finally:
                executor.shutdown()
                self.destroy_node()

        self._ros_thread = threading.Thread(target=run_executor, daemon=True)
        self._ros_thread.start()

    def shutdown(self):
        """Shutdown the node and stop ROS spinning"""
        self._shutdown.set()
        if hasattr(self, '_ros_thread'):
            self._ros_thread.join()
        self._executor.shutdown(wait=True)

    def start_mission(self):
        """Start the mission sequence"""
        self.get_logger().info('Starting drone mission...')
        try:
            self._execute_mission()
        except Exception as e:
            self.get_logger().error(f"Mission failed: {str(e)}")
            self._do_land()
        finally:
            self.shutdown()

    def _execute_mission(self):
        """Execute the complete drone mission sequence"""
        try:
            # Wait for services to be available
            self.get_logger().info('Waiting for services...')
            if not all([
                self._takeoff_client.wait_for_service(timeout_sec=5.0),
                self._land_client.wait_for_service(timeout_sec=5.0),
                self._drive_circle_client.wait_for_server(timeout_sec=5.0)
            ]):
                raise RuntimeError("Required services not available")

            # Execute takeoff
            takeoff_future = self._executor.submit(self._do_takeoff)
            if not takeoff_future.result():
                raise RuntimeError("Takeoff failed")
            
            # Wait for stabilization after takeoff
            if not self._takeoff_complete.wait(timeout=2.0):
                raise RuntimeError("Takeoff stabilization timeout")
            
            # Execute circle trajectory
            circle_future = self._executor.submit(self._do_circle_trajectory)
            if not circle_future.result():
                raise RuntimeError("Circle trajectory failed")
            
            # Wait for stabilization before landing
            if not self._circle_complete.wait(timeout=2.0):
                raise RuntimeError("Circle stabilization timeout")
            
            # Execute landing
            landing_future = self._executor.submit(self._do_land)
            if not landing_future.result():
                raise RuntimeError("Landing failed")

            self.get_logger().info("Mission completed successfully")

        except Exception as e:
            self.get_logger().error(f"Mission failed: {str(e)}")
            # Attempt emergency landing
            self._do_land()

    def _do_takeoff(self) -> bool:
        """Execute takeoff sequence"""
        self.get_logger().info(f"Initiating takeoff to {self._target_altitude}m...")
        
        request = Takeoff.Request()
        request.altitude = self._target_altitude
        
        try:
            future = self._takeoff_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is None:
                self.get_logger().error("Takeoff service call failed - no response")
                return False
                
            if not response.success:
                self.get_logger().error("Takeoff service call failed")
                return False
                
            self.get_logger().info("Takeoff successful")
            self._takeoff_complete.set()
            return True
            
        except Exception as e:
            self.get_logger().error(f"Takeoff error: {str(e)}")
            return False

    def _do_circle_trajectory(self) -> bool:
        """Execute circle trajectory sequence"""
        self.get_logger().info("Initiating circle trajectory...")
        
        try:
            # Create goal
            goal_msg = DriveCircle.Goal()
            goal_msg.radius = self._circle_radius
            goal_msg.repetitions = self._circle_repetitions
            goal_msg.time_limit = self._time_limit

            # Send goal
            send_goal_future = self._drive_circle_client.send_goal_async(
                goal_msg,
                feedback_callback=self._circle_feedback_callback
            )
            rclpy.spin_until_future_complete(self, send_goal_future)
            
            if not send_goal_future.result().accepted:
                self.get_logger().error("Circle trajectory goal rejected")
                return False

            # Get the goal handle and wait for the result
            goal_handle = send_goal_future.result()
            if goal_handle is None:
                self.get_logger().error("Circle trajectory goal handle is None")
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            get_result_response = result_future.result()

            if get_result_response is None:
                self.get_logger().error("Circle trajectory result is None")
                return False

            if not get_result_response.result.success:
                self.get_logger().error("Circle trajectory failed")
                return False

            self.get_logger().info("Circle trajectory completed successfully")
            self._circle_complete.set()
            return True
            
        except Exception as e:
            self.get_logger().error(f"Circle trajectory error: {str(e)}")
            return False

    def _circle_feedback_callback(self, feedback_msg):
        """Handle feedback from circle trajectory action"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Circle progress: iteration {feedback.current_iteration}/{self._circle_repetitions}'
        )

    def _do_land(self) -> bool:
        """Execute landing sequence"""
        self.get_logger().info("Initiating landing...")
        
        request = Land.Request()
        try:
            future = self._land_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is None:
                self.get_logger().error("Landing service call failed - no response")
                return False
                
            if not response.success:
                self.get_logger().error("Landing service call failed")
                return False
                
            self.get_logger().info("Landing successful")
            self._landing_complete.set()
            return True
            
        except Exception as e:
            self.get_logger().error(f"Landing error: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DroneMissionNode()
        node.start_ros_spin()
        node.start_mission()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()