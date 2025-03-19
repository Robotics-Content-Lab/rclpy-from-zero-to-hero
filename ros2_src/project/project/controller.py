#!/usr/bin/python3

from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from example_interfaces.msg import Float32MultiArray, Int8, Bool
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from numpy import clip


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.no_line_detected = False
        self.line_position = None
        self.line_x_value = None
        self.line_y_value = None
        self.image_width = None
        self.image_height = None
        self.rate = 0.1

        self.init_params()
        
        callback_group = ReentrantCallbackGroup()

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10, callback_group=callback_group)
        self.position_sub = self.create_subscription(
            Float32MultiArray, '/line_position', self.position_callback, 10, callback_group=callback_group)
        self.direction_sub = self.create_subscription(
            Int8, '/direction', self.direction_callback, 10, callback_group=callback_group)
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.no_line_detected_sub = self.create_subscription(
            Bool, '/no_line_detected', self.no_line_detected_callback, qos, callback_group=callback_group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(self.rate, self.timer_callback, callback_group=callback_group)
        self.add_on_set_parameters_callback(self.parameter_change_cb)
        self.get_logger().info('Controller node has been started.')

    def init_params(self):
        self.max_lin = self.declare_parameter('max_linear_speed', 0.22).value
        self.max_ang = self.declare_parameter('max_angular_speed', 0.84).value
        self.lin_kp = self.declare_parameter('linear_kp', 0.1).value
        self.ang_kp = self.declare_parameter('angular_kp', 0.008875).value
        self.control_method = self.declare_parameter('control_method', 'proportional').value

    def parameter_change_cb(self, params: List[Parameter]) -> SetParametersResult:
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == 'max_linear_speed' and param.type_ == Parameter.Type.DOUBLE:
                if param.value < 0:
                    result.successful = False
                    result.reason = 'The value of max_linear_speed must be bigger than 0.'
                else:
                    self.max_lin = param.value
            elif param.name == 'linear_kp' and param.type_ == Parameter.Type.DOUBLE:
                if abs(param.value) * self.image_width > self.max_lin:
                    result.successful = False
                    result.reason = 'The value of linear_kp must be smaller than max_linear_speed/image_width.'
                else:
                    self.lin_kp = param.value
            elif param.name == 'max_angular_speed' and param.type_ == Parameter.Type.DOUBLE:
                if param.value < 0:
                    result.successful = False
                    result.reason = 'The value of max_angular_speed must be bigger than 0.'
                else:
                    self.max_ang = param.value
            elif param.name == 'angular_kp' and param.type_ == Parameter.Type.DOUBLE:
                if abs(param.value) * (self.image_width/2) > self.max_ang:
                    result.successful = False
                    result.reason = 'With this angular kp value the control will be unstable.'
                else:
                    self.ang_kp = param.value
            elif param.name == 'control_method' and param.type_ == Parameter.Type.STRING:
                if param.value not in ['proportional', 'bangbang']:
                    result.successful = False
                    result.reason = 'The value of control_method must be either "proportional" or "bangbang".'
                else:
                    self.control_method = param.value
        return result

    def no_line_detected_callback(self, msg: Bool):
        self.no_line_detected = msg.data
        self.get_logger().info(f'No line detected: {self.no_line_detected}')

    def camera_info_callback(self, msg: CameraInfo):
        self.image_width = msg.width
        self.image_height = msg.height
        self.destroy_subscription(self.camera_info_sub)

    def position_callback(self, msg: Float32MultiArray):
        self.line_x_value = msg.data[0]
        self.line_y_value = msg.data[1]

    def direction_callback(self, msg: Int8):
        self.line_position = msg.data

    def timer_callback(self):
        msg = Twist()
        if not self.no_line_detected:
            self.get_logger().info(f'Control method: {self.control_method}', throttle_duration_sec=2.0)
            if self.control_method == 'proportional' and self.line_y_value is not None:
                self.get_logger().debug(f'Line position: \n\tx:{self.line_x_value}\n\ty:{self.line_y_value}', throttle_duration_sec=1.0)
                msg.linear.x = clip(self.lin_kp * self.line_y_value, -self.max_lin, self.max_lin)
                msg.angular.z = clip(self.ang_kp * self.line_x_value, -self.max_ang, self.max_ang)
            elif self.control_method == 'bangbang' and self.line_position is not None:
                self.get_logger().debug(f'Line position: {self.line_position}', throttle_duration_sec=1.0)
                msg.linear.x = self.max_lin
                if self.line_position == 0:
                    msg.angular.z = self.max_ang
                elif self.line_position == 1:
                    msg.angular.z = 0.0
                else:
                    msg.angular.z = -self.max_ang
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.get_logger().info(f'Linear speed: {msg.linear.x}\nAngular speed: {msg.angular.z}', throttle_duration_sec=1.0)
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
