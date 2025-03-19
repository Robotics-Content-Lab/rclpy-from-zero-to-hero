#!/usr/bin/env python3
from math import pi

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist


class OpenLoopSquareController(Node):
    def __init__(self) -> None:
        super().__init__('square_controller')

        self.declare_parameter('square_side', 2.0)
        self.declare_parameter('times', 2)

        self.square_side = self.get_parameter('square_side').value
        self.times = self.get_parameter('times').value
        self.get_logger().info(f"{self.square_side=}\n{self.times=}")
        self.max_vel = 0.22
        self.max_omega = 2.84

        self.hz = 30
        self.duration = 0.0
        self.square_state = 0
        self.state_duration = 0
        self.square_side_duration = (self.square_side / self.max_vel )* self.hz  # Compute the time needed to traverse the length of a side
        self.turn_duration = ((pi / 2 )/ self.max_omega) * self.hz  # Compute the time needed to rotate in place to take a 90 Degree turn

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        while self.pub_cmd.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for sub on {self.pub_cmd.topic} ... ", throttle_duration_sec=1.0)
        self.timer = self.create_timer(1/self.hz, self.move_in_square)

    def move_in_square(self):
        '''Open Loop Control to move the robot in a square of size self.square_side
        self.square_state == 0: go straight
        self.square_state == 1: rotate in place to the left
        '''
        speed: Twist = Twist()
        if self.duration <= self.times * (self.square_side_duration*4 + self.turn_duration*4):
            self.duration += 1
            self.get_logger().info(f'{self.duration=} <= {self.times * (self.square_side_duration*4 + self.turn_duration*4)}', throttle_duration_sec=(self.times * (self.square_side_duration*4 + self.turn_duration*4)//(10*self.hz)))
            if self.square_state == 0:
                speed.linear.x = self.max_vel
                speed.angular.z = 0.0
                self.state_duration += 1

                if self.state_duration >= self.square_side_duration:
                    self.square_state = 1
                    self.state_duration = 0
            elif self.square_state == 1:
                speed.linear.x = 0.0
                speed.angular.z = self.max_omega
                self.state_duration += 1

                if self.state_duration >= self.turn_duration:
                    self.square_state = 0
                    self.state_duration = 0
                    self.square_side_duration = self.square_side / self.max_vel * self.hz

            self.pub_cmd.publish(speed)
        else:
            self.pub_cmd.publish(Twist())
            self.timer.cancel()
            self.destroy_node()
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)

    node: OpenLoopSquareController = OpenLoopSquareController()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ROSInterruptException, SystemExit):
        node.get_logger().info("Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()