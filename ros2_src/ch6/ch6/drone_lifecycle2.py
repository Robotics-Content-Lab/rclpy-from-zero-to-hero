import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State
from std_msgs.msg import Empty, Bool, Int8, String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Range, Image, Imu

STATES = {
    0: "Landed",
    1: "Flying",
    2: "Taking off",
    3: "Landing",
}

MODES = ["velocity", "position"]


class DroneObject(LifecycleNode):
    def __init__(self, node_name: str = "drone_lifecycle"):
        super().__init__(node_name)
        self._state = STATES[0]
        self._mode = MODES[0]
        self._hover_distance = 0.0
        self.isFlying = False
        self.isPosctrl = False
        self.isVelMode = False

        self.logger = self.get_logger()

        # Publishers
        self.pubTakeOff = None
        self.pubLand = None
        self.pubReset = None
        self.pubPosCtrl = None
        self.pubCmd = None
        self.pubVelMode = None

        # Subscribers
        self.sub_sonar = None
        self.sub_imu = None
        self.sub_front_img = None
        self.sub_bottom_img = None
        self.sub_gt_pose = None
        self.sub_state = None
        self.sub_cmd_mode = None

    def on_configure(self, state: State):
        """Called when the node is transitioning to 'inactive' from 'unconfigured'."""
        self.logger.info("Configuring the Drone...")

        try:
            # Initialize all the subscriptions and get ready
            self.sub_sonar = self.create_subscription(Range, '/simple_drone/sonar', self.cb_sonar, 1024)
            self.sub_imu = self.create_subscription(Imu, '/simple_drone/imu', self.cb_imu, 1024)
            self.sub_front_img = self.create_subscription(Image, '/simple_drone/front/image_raw', self.cb_front_img, 1024)
            self.sub_bottom_img = self.create_subscription(Image, '/simple_drone/bottom/image_raw', self.cb_bottom_img, 1024)
            self.sub_gt_pose = self.create_subscription(Pose, '/simple_drone/gt_pose', self.cb_gt_pose, 1024)
            self.sub_state = self.create_subscription(Int8, '/simple_drone/state', self.cb_state, 1024)
            self.sub_cmd_mode = self.create_subscription(String, '/simple_drone/cmd_mode', self.cb_cmd_mode, 1024)
            
            # Check if all publishers and subscribers are successfully created
            if self.pubTakeOff.get_subscription_count() != 1 or self.pubLand.get_subscription_count() != 1 \
                or self.pubReset.get_subscription_count() != 1 or self.pubPosCtrl.get_subscription_count() != 1 \
                    or self.pubCmd.get_subscription_count() != 1 or self.pubVelMode.get_subscription_count() != 1:
                raise RuntimeError("Not all publishers have a subscriber")

            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.logger.error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State):
        """Called when the node is transitioning to 'active' from 'inactive'."""
        self.logger.info("Activating the Drone...")

        # Check if the drone is already flying (which should not happen during activation)
        if self.isFlying:
            self.logger.error("Drone is already flying, cannot activate!")
            return TransitionCallbackReturn.FAILURE

        self.pubTakeOff = self.create_publisher(Empty, '/simple_drone/takeoff', 1024)
        self.pubLand = self.create_publisher(Empty, '/simple_drone/land', 1024)
        self.pubReset = self.create_publisher(Empty, '/simple_drone/reset', 1024)
        self.pubPosCtrl = self.create_publisher(Bool, '/simple_drone/posctrl', 1024)
        self.pubCmd = self.create_publisher(Twist, '/simple_drone/cmd_vel', 1024)
        self.pubVelMode = self.create_publisher(Bool, '/simple_drone/dronevel_mode', 1024)

        # Further activation logic
        self.logger.info("Drone activated and ready to fly.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        """Called when the node is transitioning to 'inactive' from 'active'."""
        self.logger.info("Deactivating the Drone...")

        # If the drone is not flying, deactivation might fail
        if not self.isFlying:
            self.logger.error("Drone is not flying, cannot deactivate!")
            return TransitionCallbackReturn.FAILURE

        # The drone should stop flying and hover safely
        self.hover()
        self.isFlying = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        """Called when the node is transitioning to 'unconfigured' from 'inactive'."""
        self.logger.info("Cleaning up resources...")

        try:
            # Destroy all resources (subscriptions, publishers)
            self.destroy_subscription(self.sub_sonar)
            self.destroy_subscription(self.sub_imu)
            self.destroy_subscription(self.sub_front_img)
            self.destroy_subscription(self.sub_bottom_img)
            self.destroy_subscription(self.sub_gt_pose)
            self.destroy_subscription(self.sub_state)
            self.destroy_subscription(self.sub_cmd_mode)

            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.logger.error(f"Cleanup failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: State):
        """Called when the node is shutting down."""
        self.logger.info("Shutting down the Drone...")

        try:
            # Perform any shutdown logic
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.logger.error(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def takeOff(self):
        if self.isFlying:
            self.logger.error("Takeoff failed: Drone is already flying.")
            return False
        self.logger.info("Taking off")
        self.pubTakeOff.publish(Empty())
        self.isFlying = True
        return True

    def land(self):
        if not self.isFlying:
            self.logger.error("Landing failed: Drone is not flying.")
            return False
        self.logger.info("Landing")
        self.pubLand.publish(Empty())
        self.isFlying = False
        return True

    def hover(self):
        if not self.isFlying:
            self.logger.error("Hover failed: Drone is not flying.")
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

    # Callbacks
    def cb_sonar(self, msg: Range):
        self._sonar = msg
        self._hover_distance = msg.min_range

    def cb_imu(self, msg: Imu):
        self._imu = msg

    def cb_front_img(self, msg: Image):
        self._front_img = msg

    def cb_bottom_img(self, msg: Image):
        self._bottom_img = msg

    def cb_gt_pose(self, msg: Pose) -> None:
        self._gt_pose = msg

    def cb_state(self, msg: Int8):
        self._state = STATES[msg.data]
        self.logger.info("State: {}".format(self._state), throttle_duration_sec=1)

    def cb_cmd_mode(self, msg: String):
        if msg.data in MODES:
            self._mode = msg.data
            self.logger.info("Changed command mode to: {}".format(self._mode))
        else:
            self.logger.error("Invalid command mode: {}".format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    lifecycle_node = DroneObject()
    rclpy.spin(lifecycle_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
