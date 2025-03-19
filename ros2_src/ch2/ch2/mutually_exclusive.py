import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

class MutuallyExclusive(Node):
    def __init__(self):
        super().__init__('robot_controller')
        sim_time_parameter = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_parameter])
        self.WHEEL_RADIUS = 0.33  # [m]
        self.last_encoder_state = None
        self.encoder_distance = 0.0  # [m]
        self.imu_distance = 0.0  # [m]
        self.last_imu_time = None
        self.traveled_distance = 0.0  # [m]
        self.callback_cnt = 0

        self.callback_group = MutuallyExclusiveCallbackGroup()  # Create a mutually exclusive callback group

        # Subscription to JointState (wheel encoders)
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group  # Assign the callback group to the subscription
        )

        # Subscription to IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10,
            callback_group=self.callback_group  # Assign the callback group to the subscription
        )

        # Publisher for movement commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Timer for periodically executing movement commands
        self.command_timer = self.create_timer(
            1.0,  # Interval in seconds
            self.command_callback,
            callback_group=self.callback_group  # Assign the callback group to the subscription
        )

        # Timer to average the distances from IMU and encoders
        self.update_distance_timer = self.create_timer(
            1.0,  # Interval in seconds
            self.update_distance,
            callback_group=self.callback_group  # Assign the callback group to the subscription
        )

    def joint_state_callback(self, msg: JointState):
        self.callback_cnt += 1
        if self.last_encoder_state == None:
            self.last_encoder_state = msg.position
            return
        # Calculate the average wheel displacement
        wheel_distance = ((msg.position[0] - self.last_encoder_state[0]) + (msg.position[1] - self.last_encoder_state[1])) / 2
        self.last_encoder_state = msg.position
        self.encoder_distance += wheel_distance * self.WHEEL_RADIUS
        self.get_logger().info(f'[{self.callback_cnt=}]: Wheel distance updated: {self.encoder_distance}')

    def imu_callback(self, msg: Imu):
        self.callback_cnt += 1
        current_time = msg.header.stamp
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return
        imu_dt = (current_time.sec - self.last_imu_time.sec) + (current_time.nanosec - self.last_imu_time.nanosec) * 1e-9
        self.last_imu_time = current_time  # Update last IMU time

        # Integration of acceleration to get velocity
        # Assuming movement only along x axis and initial velocity is zero for simplicity and resets every time interval
        accel = msg.linear_acceleration.x
        distance = 0.5 * accel * (imu_dt ** 2)  # s = 0.5 * a * t^2
        self.imu_distance += distance
        self.get_logger().info(f'[{self.callback_cnt=}]: IMU distance updated: {self.imu_distance}')

    def update_distance(self):
        self.callback_cnt += 1
        incremental_distance = (self.encoder_distance + self.imu_distance) / 2
        self.traveled_distance += incremental_distance

        # Reset distance measurements
        self.encoder_distance = 0
        self.imu_distance = 0
        self.get_logger().info(f'[{self.callback_cnt=}]: Traveled distance updated to: {self.traveled_distance}')

    def command_callback(self):
        self.callback_cnt += 1
        # Generate and publish commands inversely proportional to traveled distance
        speed = 0.22 / max(1, self.traveled_distance)  # Prevent division by zero
        speed = speed if speed > 0.05 else 0.0
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'[{self.callback_cnt=}]: Published command with linear.x = {twist.linear.x}')
        if speed == 0.0:
            self.get_logger().info('Stopping the robot')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    robot_controller = MutuallyExclusive()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
