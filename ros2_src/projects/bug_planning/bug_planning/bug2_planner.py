#!/usr/bin/env python3

from math import atan2, sqrt, hypot, radians, degrees, fabs

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.exceptions import ROSInterruptException

from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from bug_planning.utils.transforms import euler_from_quaternion, normalize
from bug_planning.utils.visualization import (
    create_goal_marker,
    delete_goal_marker,
    draw_line,
)


class Controller(Node):
    ODOM_TOPIC = "odom"
    GOAL_TOPIC = "goal_pose"
    CMD_TOPIC = "cmd_vel"
    MARKER_TOPIC = "goal_marker"
    SCAN_TOPIC = "scan"

    ROBOT_MODES = ["GO_TO_GOAL_MODE", "FOLLOW_WALL_MODE", "OBSTACLE_AVOIDANCE_MODE"]
    PLANNER_STATES = ["INITIALIZING", "ADJUST_HEADING", "GO_STRAIGHT", "AT_GOAL"]
    WALL_FOLLOWING_STATE = ["TURNING_LEFT", "SEARCHING_FOR_WALL", "FOLLOWING_WALL"]

    def __init__(self, constant_vel: bool = None) -> None:
        super().__init__("controller")
        client_cb_group = ReentrantCallbackGroup()
        timer_cb_group = None

        self.cb_logger = rclpy.logging.get_logger("cb_logger")
        self.timer_logger = rclpy.logging.get_logger("timer_logger")

        """Initializes the class member variables"""
        self.x: float = 0.0  # m
        self.y: float = 0.0  # m
        self.theta: float = 0.0  # rad

        """Goal specific cariables"""
        self.goal: Point = None
        self.goal_dist_tolerance: float = 0.02
        self.goal_heading_tolerance: float = radians(10)

        """Motion planner specific variables"""
        self.constant_vel: bool = constant_vel
        self.max_vel: float = 0.22
        self.slow_vel: float = 0.22 / 4
        self.max_omega: float = 2.84
        self.slow_omega: float = 0.1

        # ToDo: Dynamic Reconfigure
        self.k_rho: float = 0.3
        self.k_alpha: float = 0.8
        self.k_beta: float = -0.15

        """Obstacle detection specific variables"""
        # Define the angles for each direction
        self.left_angle: float = radians(90)
        self.front_left_angle: float = radians(45)
        self.front_angle: float = 0
        self.front_right_angle: float = radians(315)
        self.right_angle: float = radians(270)

        # Define the range of angles to search for obstacles
        self.angle_range: float = radians(5)

        # Initialize the closest obstacle distances
        self.closest_obstacle_left: float = float("inf")
        self.closest_obstacle_front_left: float = float("inf")
        self.closest_obstacle_front: float = float("inf")
        self.closest_obstacle_front_right: float = float("inf")
        self.closest_obstacle_right: float = float("inf")

        """BUG2 planner specific variables"""
        self.robot_mode: str = "GO_TO_GOAL_MODE"
        self.planner_state: str = "INITIALIZING"
        self.wall_following_state: str = "TURNING_LEFT"

        # M-Line Parameters
        self.m_line_calculated: bool = False  # Start to Goal line calculated?
        self.m_line_slope: float = 0.0
        self.m_line_y_intercept: float = 0.0
        self.m_line_start_x: float = 0.0
        self.m_line_start_y: float = 0.0
        self.m_line_goal_x: float = 0.0
        self.m_line_goal_y: float = 0.0

        # Wall Follower Parameters. ToDO: Make Dynamic Reconfigure
        self.wall_dist_thresh: float = (
            0.25  # Anything less then this distance means we encountered a wall
        )
        self.wall_too_close: float = 0.15  # Too close to wall threshold for right side wall following
        self.yaw_precision: float = radians(2.0)  # +/- 2 degree threshhold on yaw
        self.dist_precision: float = 0.2  # m threshold for goal reached
        self.dist_to_m_line_thresh: float = 0.4  # Leave point, of WALL_FOLLOWING_MODE, must be within the threshold to change to GO_TO_GOAL_MODE
        self.hit_point: Point = Point(
            x=0.0, y=0.0, z=0.0
        )  # Point that depicts the coordinates where the robot hits a wall
        self.leave_point: Point = Point(
            x=0.0, y=0.0, z=0.0
        )  # Point that depicts the coordinates where the robot leaves a wall
        self.distance_to_goal_from_hit_point = (
            0.0  # Distance between the hit point and the goal in meters
        )
        self.distance_to_goal_from_leave_point = (
            0.0  # Distance between the leave point and the goal in meters
        )
        self.leave_point_to_hit_point_diff = 0.25  # Min travel distance along wall to allow robot_mode switch. Prevents the robot from getting stuck and rotating in endless circles.

        """ROS specific variables"""
        self.timer = self.create_timer(1 / 30, self.bug2, callback_group=timer_cb_group)
        self.sub_odom = self.create_subscription(
            Odometry,
            Controller.ODOM_TOPIC,
            lambda msg: self.odom_callback(msg),
            10,
            callback_group=client_cb_group,
        )
        self.sub_goal = self.create_subscription(
            PoseStamped,
            Controller.GOAL_TOPIC,
            lambda msg: self.clicked_point_callback(msg),
            2,
            callback_group=client_cb_group,
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            Controller.SCAN_TOPIC,
            lambda msg: self.scan_callback(msg),
            qos_profile_sensor_data,
            callback_group=client_cb_group,
        )
        self.pub_cmd = self.create_publisher(Twist, Controller.CMD_TOPIC, 10)
        self.pub_goal_vis = self.create_publisher(Marker, Controller.MARKER_TOPIC, 10)

        self.get_logger().info("Finished init node")

    def _set_robot_mode(self, mode: str) -> None:
        try:
            assert mode in Controller.ROBOT_MODES
            self.robot_mode = mode
            self.get_logger().info(f"Changing robot mode to: {self.robot_mode}")
        except Exception as e:
            self.get_logger().error(f"Could not change robot_mode to: {mode}")
            self.get_logger().error(e)

    def _set_planner_states(self, state: str) -> None:
        try:
            assert state in Controller.PLANNER_STATES
            self.planner_state = state
            self.get_logger().info(f"Changing planner state to: {self.planner_state}")
        except Exception as e:
            self.get_logger().error(f"Could not change planner_state to: {state}")
            self.get_logger().error(e)

    def _set_wall_following_state(self, state: str) -> None:
        try:
            assert state in Controller.WALL_FOLLOWING_STATE
            self.wall_following_state = state
            self.get_logger().info(f"Changing wall following state to: {self.wall_following_state}")
        except Exception as e:
            self.get_logger().error(f"Could not change wall_following_state to: {state}")
            self.get_logger().error(e)

    def odom_callback(self, msg: Odometry) -> None:
        """Callback function for the self.sub ROS subscriber

        Args:
            msg (Odometry): The odometry of the robot.
            Odometry is the measurement of wheel rotation using optical encoders, similar to the odometer on your automobile.
                Odometry isn't always as precise as one would want, yet it's the backbone of robot tracking.
                It can be found on all robots, from low-cost toys for kids to multi-million-dollar machines.
                Odometry is utilized as a complement or backup even when alternatives to Odometry are used (IMU, GNSS).
                Some Odometry alternatives are not easily transportable to other sites or may not operate in specific situations or settings.
            Odometry has the potential to be a very helpful tool.
            Just don't hold your breath for perfection.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # ROS uses quaternions to describe the roll pitch and yaw, thus we must convert it to euler angles
        quat = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion(quat)

        self.cb_logger.debug(
            f"Current Pose: {self.x=}, {self.y=}, {degrees(self.theta)=}",
            throttle_duration_sec=0.5,
        )

    def clicked_point_callback(self, msg: PoseStamped) -> None:
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        (_, _, goal_th) = euler_from_quaternion(msg.pose.orientation)

        self.goal: Point = Point(x=goal_x, y=goal_y, z=goal_th)
        self.cb_logger.info(f"Current Goal: {goal_x=}, {goal_y=}, {degrees(goal_th)=}")
        self.pub_goal_vis.publish(create_goal_marker(msg))

    def scan_callback(self, msg: LaserScan) -> None:
        """This function uses the range and angle data from the LaserScan message to calculate
           the closest obstacles in the left, front left, front, front right, and right directions.
           It defines the angles for each direction, as well as the range of angles to search for obstacles.
           It also initializes the closest obstacle distances for each direction.
           It iterates over the range and angle data, determining the closest obstacle for each direction.
        Args:
            msg (LaserScan): The message containing the laser scan data.
        """
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Initialize the closest obstacle distances
        closest_obstacle_left = float("inf")
        closest_obstacle_front_left = float("inf")
        closest_obstacle_front = float("inf")
        closest_obstacle_front_right = float("inf")
        closest_obstacle_right = float("inf")

        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment

            if (
                (self.left_angle - self.angle_range)
                <= angle
                <= (self.left_angle + self.angle_range)
            ):
                if ranges[i] < closest_obstacle_left:
                    closest_obstacle_left = ranges[i]
            elif (
                (self.front_left_angle - self.angle_range)
                <= angle
                <= (self.front_left_angle + self.angle_range)
            ):
                if ranges[i] < closest_obstacle_front_left:
                    closest_obstacle_front_left = ranges[i]
            elif (
                (self.front_angle - self.angle_range)
                <= angle
                <= (self.front_angle + self.angle_range)
            ):
                if ranges[i] < closest_obstacle_front:
                    closest_obstacle_front = ranges[i]
            elif (
                (self.front_right_angle - self.angle_range)
                <= angle
                <= (self.front_right_angle + self.angle_range)
            ):
                if ranges[i] < closest_obstacle_front_right:
                    closest_obstacle_front_right = ranges[i]
            elif (
                (self.right_angle - self.angle_range)
                <= angle
                <= (self.right_angle + self.angle_range)
            ):
                if ranges[i] < closest_obstacle_right:
                    closest_obstacle_right = ranges[i]

        self.closest_obstacle_left = closest_obstacle_left
        self.closest_obstacle_front_left = closest_obstacle_front_left
        self.closest_obstacle_front = closest_obstacle_front
        self.closest_obstacle_front_right = closest_obstacle_front_right
        self.closest_obstacle_right = closest_obstacle_right

        self.cb_logger.debug(
            "Closest obstacle:\n\tleft: {},\n\tfront left: {},\n\tfront: {},\n\tfront right: {},\n\tright: {}".format(
                self.closest_obstacle_left,
                self.closest_obstacle_front_left,
                self.closest_obstacle_front,
                self.closest_obstacle_front_right,
                self.closest_obstacle_right,
            )
        )

    def bug2(self) -> None:
        if self.goal == None:
            self.timer_logger.info("Waiting for goal", throttle_duration_sec=1)
            return
        if self.x == None or self.y == None or self.theta == None:
            self.timer_logger.info(
                "Waiting to retrieve current pose", throttle_duration_sec=1
            )
            return

        if self.m_line_calculated == False:
            self.m_line_start_x = self.x
            self.m_line_start_y = self.y
            self.m_line_goal_x = self.goal.x
            self.m_line_goal_y = self.goal.y

            # Calculate the slope of the m-line from start to goal
            self.m_line_slope = (self.m_line_goal_y - self.m_line_start_y) / (
                self.m_line_goal_x - self.m_line_start_x
            )
            # Solve for the intercept b
            self.m_line_y_intercept = self.m_line_goal_y - (
                self.m_line_slope * self.m_line_goal_x
            )
            # Draw m-line
            self.pub_goal_vis.publish(
                draw_line(Point(x=self.x, y=self.y, z=0.0), self.goal)
            )

            self._set_robot_mode("GO_TO_GOAL_MODE")
            self._set_planner_states("ADJUST_HEADING")
            self.m_line_calculated = True

        if self.robot_mode == "GO_TO_GOAL_MODE":
            self.go_to_goal()
            # self.go_to()
        elif self.robot_mode == "FOLLOW_WALL_MODE":
            self.move_robot()
            self.follow_wall()
        else:
            pass

    def follow_wall(self) -> None:
        self.get_logger().info("Following wall ... ", throttle_duration_sec=1)
        linear_x = 0.0
        angular_z = 0.0
        # Calculate the point on the start-goal
        # line that is closest to the current position
        x_start_goal_line = self.x
        y_start_goal_line = (self.m_line_slope * (x_start_goal_line)) + (
            self.m_line_y_intercept
        )

        # Calculate the distance between current position
        # and the start-goal line
        distance_to_start_goal_line = sqrt(
            pow(x_start_goal_line - self.x, 2) + pow(y_start_goal_line - self.y, 2)
        )

        # If we hit the start-goal line again
        if distance_to_start_goal_line < self.dist_to_m_line_thresh:

            # Determine if we need to leave the wall and change the mode
            # to 'go to goal'
            # Let this point be the leave point
            self.leave_point_x = self.x
            self.leave_point_y = self.y

            # Record the distance to the goal from the leave point
            self.distance_to_goal_from_leave_point = sqrt(
                pow(self.goal.x - self.leave_point_x, 2)
                + pow(self.goal.y - self.leave_point_y, 2)
            )

            # Is the leave point closer to the goal than the hit point?
            # If yes, go to goal.
            diff = (
                self.distance_to_goal_from_hit_point
                - self.distance_to_goal_from_leave_point
            )
            if diff > self.leave_point_to_hit_point_diff:
                # Change the mode. Go to goal.
                self._set_robot_mode("GO_TO_GOAL_MODE")
                return

        # Logic for following the wall
        # >d means no wall detected by that laser beam
        # <d means an wall was detected by that laser beam
        d = self.wall_dist_thresh

        if (
            self.closest_obstacle_front_left > d
            and self.closest_obstacle_front > d
            and self.closest_obstacle_front_right > d
        ):
            self.timer_logger.info("turn right to find wall 1")
            self._set_wall_following_state("SEARCHING_FOR_WALL")
            linear_x = self.slow_vel
            angular_z = -self.slow_omega
        elif (
            self.closest_obstacle_front_left > d
            and self.closest_obstacle_front < d
            and self.closest_obstacle_front_right > d
        ):
            self.timer_logger.info("obstacle ahead, turning left")
            self._set_wall_following_state("TURNING_LEFT")
            angular_z = self.max_omega
        elif (
            self.closest_obstacle_front_left > d
            and self.closest_obstacle_front > d
            and self.closest_obstacle_front_right < d
        ):
            if self.closest_obstacle_front_right < self.wall_too_close:
                self.timer_logger.info("Getting too close to the wall")
                self._set_wall_following_state("TURNING_LEFT")
                linear_x = self.slow_vel
                angular_z = self.slow_vel
            else:
                self.timer_logger.info("Go straight ahead")
                self._set_wall_following_state("FOLLOWING_WALL")
                linear_x = self.max_vel
        elif (
            self.closest_obstacle_front_left < d
            and self.closest_obstacle_front > d
            and self.closest_obstacle_front_right > d
        ):
            self.timer_logger.info("turn right to find wall 2")
            self._set_wall_following_state("SEARCHING_FOR_WALL")
            linear_x = self.slow_vel
            angular_z = -self.slow_omega 
        elif (
            self.closest_obstacle_front_left > d
            and self.closest_obstacle_front < d
            and self.closest_obstacle_front_right < d
        ):
            self.timer_logger.info("obstacle ahead and front right, turning left")
            self._set_wall_following_state("TURNING_LEFT")
            angular_z = self.max_omega

        elif (
            self.closest_obstacle_front_left < d
            and self.closest_obstacle_front < d
            and self.closest_obstacle_front_right > d
        ):
            self.timer_logger.info("obstacle ahead and front left, turning left")
            self._set_wall_following_state("TURNING_LEFT")
            angular_z = self.max_omega

        elif (
            self.closest_obstacle_front_left < d
            and self.closest_obstacle_front < d
            and self.closest_obstacle_front_right < d
        ):
            self.timer_logger.info("obstacle ahead and front left/right, turning left")
            self._set_wall_following_state("TURNING_LEFT")
            angular_z = self.max_omega

        elif (
            self.closest_obstacle_front_left < d
            and self.closest_obstacle_front > d
            and self.closest_obstacle_front_right < d
        ):
            self.timer_logger.info("obstacle front left/right, turning left")
            self._set_wall_following_state("SEARCHING_FOR_WALL")
            linear_x = self.slow_vel
            angular_z = -self.slow_omega  # turn right to find wall
        else:
            self.timer_logger.info("In else of wall following")
            pass

        # Send velocity command to the robot
        self.move_robot(linear_x, angular_z)
        pass

    def go_to_goal(self) -> None:
        self.get_logger().info("Started going to goal")

        # Check if a wall is too close up front
        d = self.wall_dist_thresh
        if (
            self.closest_obstacle_front_left < d
            or self.closest_obstacle_front < d
            or self.closest_obstacle_front_right < d
        ):

            self._set_robot_mode("FOLLOW_WALL_MODE")

            # Record hit point of wall
            self.hit_point.x = self.x
            self.hit_point.y = self.y

            # Record the distance to the goal from the hit point
            # self.distance_to_goal_from_hit_point = sqrt(
            #     (pow(self.goal.x - self.hit_point.x, 2))
            #     + (pow(self.goal.y - self.hit_point.y, 2))
            # )
            self.distance_to_goal_from_hit_point = hypot(
                (self.goal.x - self.hit_point.x), (self.goal.y - self.hit_point.y)
            )

            # Take a hard left to avoid obstacle
            self.move_robot(linear_x=0.0, angular_z=self.slow_omega*2)

            return

        # Fix the heading
        if self.planner_state == "ADJUST_HEADING":
            # Calculate the desired heading based on the current position
            # and the desired position
            desired_yaw = atan2(self.goal.y - self.y, self.goal.x - self.x)

            # How far off is the current heading in radians?
            yaw_error = desired_yaw - self.theta

            # Adjust heading if heading is not good enough
            if fabs(yaw_error) > self.yaw_precision:
                if yaw_error > 0:
                    # Turn left (counterclockwise)
                    angular_z = self.slow_omega
                else:
                    # Turn right (clockwise)
                    angular_z = -self.slow_omega
                # Command the robot to adjust the heading
                self.move_robot(angular_z=angular_z)

            # Change the state if the heading is good enough
            else:
                # Change the state
                self._set_planner_states("GO_STRAIGHT")
                # Command the robot to stop turning
                self.move_robot()

        elif self.planner_state == "GO_STRAIGHT":
            position_error = sqrt(
                pow(self.goal.x - self.x, 2) + pow(self.goal.y - self.y, 2)
            )
            position_error = hypot((self.goal.x - self.x), (self.goal.y - self.y))

            # If we are still too far away from the goal
            if position_error > self.dist_precision:
                # Move straight ahead
                linear_x = self.max_vel
                # Command the robot to move
                self.move_robot(linear_x=linear_x)

                # Check our heading
                desired_yaw = atan2(self.goal.y - self.y, self.goal.x - self.x)

                # How far off is the heading?
                yaw_error = desired_yaw - self.theta

                # Check the heading and change the state if there is too much heading error
                if fabs(yaw_error) > self.yaw_precision:
                    # Change the state
                    self._set_planner_states("ADJUST_HEADING")

            # We reached our goal. Change the state.
            else:
                # Change the state
                self._set_planner_states("AT_GOAL")
                self.goal = None
                self.m_line_calculated = False
                # Command the robot to stop
                self.stop_robot()

        elif self.planner_state == "AT_GOAl":
            self.goal = None
            self.m_line_calculated = False
            self._set_planner_states("INITIALIZING")
            pass
        else:
            pass

    def go_to(self) -> None:
        """Calculates the current distance (rho) to the goal_point as well as the angle to the goal.
        Based on the delta angle to the goal this function decides wether to rotate the robot in place or go straight ahead.

        Args:
            goal_point (Point): [description]
        """

        def clamp(num, max_vel):
            """Clamp the input between -max_vel and +max_vel"""
            return max(min(num, max_vel), -max_vel)

        if self.goal == None:
            return

        # speed: Twist = Twist()

        delta_x = self.goal.x - self.x
        delta_y = self.goal.y - self.y
        delta_th = self.goal.z - self.theta
        theta = self.theta

        rho = sqrt(delta_x**2 + delta_y**2)
        self.timer_logger.info(
            f"Distance to goal= {rho:.2f}m\nHeading difference= {degrees(delta_th):.3f}",
            throttle_duration_sec=1,
        )

        alpha = normalize(atan2(delta_y, delta_x) - theta)
        beta = normalize(degrees(theta) - atan2(delta_y, delta_x))
        # beta = self.goal.z

        v = clamp(self.k_rho * rho, 0.22)
        omega = clamp(self.k_alpha * alpha + self.k_beta * beta, 2.22)

        if self.constant_vel:
            abs_v = abs(v)
            v = v / abs_v * self.max_vel
            omega = omega / abs_v * self.max_omega

        # speed.linear.x = v
        # speed.angular.z = omega
        self.move_robot(linear_x=v, angular_z=omega)
        # self.pub_cmd.publish(speed)

        if (
            rho < self.goal_dist_tolerance
            and abs(delta_th) < self.goal_heading_tolerance
        ):
            position_error = hypot(self.goal.x - self.x, self.goal.y - self.y)
            heading_error = abs(self.goal.z - self.theta)
            self.timer_logger.info(
                f"Final position error: {position_error:.2f}m\nFinal orientation error: {degrees(heading_error):.2f}"
            )
            self.goal = None
            self.stop_robot()

    def move_robot(self, linear_x=0.0, angular_z=0.0):
        msg: Twist = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub_cmd.publish(msg)
        self.timer_logger.debug(f"Publishing {msg=}", throttle_duration_sec=0.5)

    def stop_robot(self) -> None:
        """This function stops the robot"""
        self.get_logger().warn("Stopping robot")
        self.pub_goal_vis.publish(delete_goal_marker(1))
        self.pub_goal_vis.publish(delete_goal_marker(2))
        speed: Twist = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pub_cmd.publish(speed)


def main(args=None):
    # Initialize rclpy library
    rclpy.init(args=args)
    simple_pose_mover: Controller = Controller()
    simple_pose_mover.get_logger().set_level(LoggingSeverity.DEBUG)

    executor = MultiThreadedExecutor()
    executor.add_node(simple_pose_mover)

    try:
        executor.spin()
    except (KeyboardInterrupt, ROSInterruptException) as e:
        simple_pose_mover.get_logger().error(e)
        simple_pose_mover.stop_robot()
    finally:
        simple_pose_mover.stop_robot()
        simple_pose_mover.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
