import threading  # Import threading to enable graceful shutdown

import rclpy
from rclpy.executors import SingleThreadedExecutor

from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn  # Import the LifecycleNode class and TransitionCallbackReturn enum
from lifecycle_msgs.msg import State  # Import the State message from the lifecycle_msgs package
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class DroneObject(LifecycleNode):  # Define a new class that extends the LifecycleNode class.
    def __init__(self, node_name: str = "drone_lifecycle"):
        super().__init__(node_name)
        # Initialize event signaling to gracefully shut down the node
        self.done_event = threading.Event()

        # Track the current state of the node
        self.state = State.PRIMARY_STATE_UNKNOWN

        # Indicates whether the drone is currently flying or not
        self.isFlying = False
        self.droneNS = self.declare_parameter('droneNS', '').value  # Declare a parameter for the drone namespace

        self.logger = self.get_logger()

        # Initialize publisher and subscriber handles to None
        self.pubTakeOff = None
        self.pubLand = None
        self.pubReset = None
        self.pubPosCtrl = None
        self.pubCmd = None
        self.pubVelMode = None
        self.sub_takeoff_cmd = None
        self.sub_land_cmd = None
        self.sub_cmd_vel = None

    def _init_subscribers(self):
        self.sub_takeoff_cmd = self.create_subscription(Empty, '/takeoff', self.cb_takeoff_cmd, 1024)
        self.sub_land_cmd = self.create_subscription(Empty, '/land', self.cb_land_cmd, 1024)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 1024)

    def on_configure(self, state: State):
        """
        Called when the node transitions from 'unconfigured' to 'inactive'.
        - This phase is for preparing the node, like setting up parameters and subscribers.
        """
        self.state = state
        self.logger.info(f"Configuring the Drone, changing from: {state.label}")
        self._cleanup_subscribers()  # Clean up any existing subscribers before re-initializing

        ns = self.get_parameter('droneNS').value  # Get the 'droneNS' parameter value
        if ns and ns != '':
            self.droneNS = ns.replace('~', '').replace('/', '')  # Set the namespace for the drone
            self.logger.info("Subscribing to topics...")
            self._init_subscribers()  # Initialize subscribers for takeoff, land, and velocity commands
            self.logger.info("Drone configured.")
            return TransitionCallbackReturn.SUCCESS  # Indicate successful configuration
        else:
            self.logger.error("Drone namespace not set. Please set the 'droneNS' parameter.")
            return TransitionCallbackReturn.ERROR  # Fail the transition due to missing namespace

    def _init_publishers(self):
        self.pubTakeOff = self.create_publisher(Empty, self.droneNS + '/takeoff', 1024)
        self.pubLand = self.create_publisher(Empty, self.droneNS + '/land', 1024)
        self.pubCmd = self.create_publisher(Twist, self.droneNS + '/cmd_vel', 1024)

    def on_activate(self, state: State):
        """
        Called when the node transitions from 'inactive' to 'active'.
        - Here the publishers are initialized, and the node is ready for active use.
        """
        def _check_sub_count(publisher):
            # Helper function to check if publishers have subscribers
            return publisher.get_subscription_count() > 0

        self.state = state
        self.logger.info(f"Activating the Drone, changing from: {state.label}")
        self._cleanup_publishers()  # Clean up any existing publishers before re-initializing
        
        self.logger.info("Initializing publishers...")
        self._init_publishers()  # Initialize publishers for takeoff, land, and velocity commands
        if not (_check_sub_count(self.pubTakeOff) and _check_sub_count(self.pubLand) and _check_sub_count(self.pubCmd)):
            self.logger.error("Not all publishers have subscribers.")
            return TransitionCallbackReturn.ERROR  # Fail the transition if publishers do not have subscribers
        
        # Node is now fully active and ready to control the drone
        self.logger.info("Drone activated, publishers initialized.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        """
        Called when the node transitions from 'active' to 'inactive'.
        - The drone stops flying and enters an idle state.
        """
        self.state = state
        self.logger.info(f"Deactivating the Drone, changing from: {state.label}")
        self.hover()
        self.logger.info("Drone deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def _cleanup_subscribers(self):
        # Helper method to destroy and clean up all subscribers
        if self.sub_takeoff_cmd:
            self.destroy_subscription(self.sub_takeoff_cmd)
        if self.sub_land_cmd:
            self.destroy_subscription(self.sub_land_cmd)
        if self.sub_cmd_vel:
            self.destroy_subscription(self.sub_cmd_vel)
        self.sub_takeoff_cmd = None
        self.sub_land_cmd = None
        self.sub_cmd_vel = None

    def _cleanup_publishers(self):
        # Helper method to destroy and clean up all publishers
        if self.pubTakeOff:
            self.destroy_publisher(self.pubTakeOff)
        if self.pubLand:
            self.destroy_publisher(self.pubLand)
        if self.pubCmd:
            self.destroy_publisher(self.pubCmd)
        self.pubTakeOff = None
        self.pubLand = None
        self.pubCmd = None

    def on_cleanup(self, state: State):
        """
        Called when the node transitions from 'inactive' to 'unconfigured'.
        - The node cleans up resources like subscribers and publishers.
        """
        self.state = state
        self.logger.info(f"Cleaning up resources, changing from: {state.label}")
        self.logger.info("Cleaning up subscribers...")
        self._cleanup_subscribers()
        self.logger.info("Cleaning up publishers...")
        self._cleanup_publishers()
        self.logger.info("Drone cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        """
        Called when the node transitions to 'finalized'.
        - This is the final state where resources are released and the drone shuts down safely.
        """
        self.state = state
        self.logger.info(f"Shutting down the Drone, changing from: {state.label}")
        self.logger.info("Landing the drone...")
        self.land(Empty())
        self.logger.info("Cleaning up subscribers and publishers...")
        self._cleanup_subscribers()
        self._cleanup_publishers()
        self.logger.info("Drone shutdown.")
        self.done_event.set()  # Signal that the node is fully shut down
        return TransitionCallbackReturn.SUCCESS

    # Callbacks for takeoff, land, and velocity commands
    def cb_takeoff_cmd(self, msg: Empty):
        self.takeOff(msg)

    def cb_land_cmd(self, msg: Empty):
        self.land(msg)

    def cb_cmd_vel(self, msg: Twist):
        self.logger.info("Moving the drone", throttle_duration_sec=5)
        self.pubCmd.publish(msg)

    # Publish functions for controlling the drone
    def takeOff(self, msg: Empty):
        if self.isFlying:
            return False
        self.logger.info("Taking off")
        self.pubTakeOff.publish(msg)
        self.isFlying = True
        return True

    def land(self, msg: Empty):
        if not self.isFlying:
            self.logger.info("Drone is already on the ground.")
            return True
        self.logger.info("Landing")
        self.pubLand.publish(msg)
        self.isFlying = False
        return True

    def hover(self):
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()  # Use a single-threaded executor
    lifecycle_node = DroneObject()  # Instantiate the drone's lifecycle node
    executor.add_node(lifecycle_node)

    try:
        # Spin the executor in a separate thread to enable graceful shutdown
        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()

        # Wait for the node to signal that it's done and ready to shut down
        lifecycle_node.done_event.wait()

    finally:
        # Cleanup upon shutdown
        executor.shutdown()
        lifecycle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
