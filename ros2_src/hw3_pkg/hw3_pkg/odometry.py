import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import matplotlib.pyplot as plt

def create_star() :
    num_vertices = 5
    
    angles = np.linspace(0, 2 * np.pi, num_vertices, endpoint=False)
    
    x = np.cos(angles)
    y = np.sin(angles)
    
    indices = [0, 2, 4, 1, 3, 0]
    star_x = x[indices]
    star_y = y[indices]
    return star_x, star_y

class VelocityOdometry(Node):
    def __init__(self):
        super().__init__('odometry')
        odometry_cb_group = ReentrantCallbackGroup()
        ground_truth_cb_group = ReentrantCallbackGroup()

        # Initialize pose as a numpy array [x, y, theta]
        # x and y represent the robot's position in the world frame
        # theta represents the robot's orientation in the world frame
        self.vel_pose = np.array([0.0, 0.0, 0.0])
        self.kin_pose = np.array([0.0, 0.0, 0.0])

        # Initialize last_time to None
        self.last_time = None

        # Initialize joint_states to None for kinematic model
        self.joint_states = None

        # Set constants
        # WHEEL_RADIUS: The radius of the robot's wheels
        # WHEEL_DISTANCE: The distance between the robot's wheels
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_DISTANCE = 0.160

        # Subscribe to cmd_vel for the velocity model
        self.sub_cmd = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_model,
            10,
            callback_group=odometry_cb_group
        )
        # Subscribe to joint_states topic for the kinematic model
        self.sub_joint_states = self.create_subscription(
            JointState,
            'joint_states',
            self.kinematic_model,
            10,
            callback_group=odometry_cb_group
        )

        # Subscribe to the ground truth odometry topic
        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_cb,
            10,
            callback_group=ground_truth_cb_group
        )

        # Initialize the plot for visualizing the robot's odometry
        self.init_plot()

    def init_plot(self):
        star_x, star_y = create_star()
        # Set up pyplot for visualization
        # plt.ion() allows for interactive plots
        plt.ion()

        # Create a new figure and axes for the plot
        self.fig, self.ax1 = plt.subplots()

        # Set the title and labels for the plot
        self.ax1.set_title("Robot Odometry")
        self.ax1.set_xlabel("y [m]")
        self.ax1.set_ylabel("x [m]")

        # Set the aspect ratio of the plot to be equal. This means that units on the x and y axes are the same size.
        self.ax1.set_aspect('equal')
        self.ax1.invert_xaxis()

        # Initialize lists for storing odometry data
        self.vel_odom_y = []
        self.vel_odom_x = []
        self.kin_odom_y = []
        self.kin_odom_x = []
        self.received_odom_y = []
        self.received_odom_x = []

        # Plot the actual star shape
        self.ax1.plot(star_y, star_x, 'k-', label="Desired Path")

        # Plot placeholders
        self.received_odom_plot, = self.ax1.plot([], [], 'ko', markerfacecolor='none', alpha=0.5, label='Ground Truth [/odom]')
        self.vel_odom_plot, = self.ax1.plot([], [], 'r-', label='Velocity Model')
        self.kin_odom_plot, = self.ax1.plot([], [], 'b-', label='Kinematic Model')

        self.ax1.legend(loc='upper left', bbox_to_anchor=(1, 1))


    def odom_cb(self, msg: Odometry):
        # Extract position from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Save the received odometry
        self.received_odom_y.append(y)
        self.received_odom_x.append(x)

        # Update plot data
        self.received_odom_plot.set_xdata(self.received_odom_y)
        self.received_odom_plot.set_ydata(self.received_odom_x)

        # Redraw the plot
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.fig.canvas.draw()
        plt.pause(0.001)


    def kinematic_model(self, msg: JointState):
        # If this is the first JointState message we've received, just store it and return
        if self.joint_states is None:
            self.joint_states = msg
            return

        position_diff = np.array(msg.position) - np.array(self.joint_states.position)
        rho = self.WHEEL_RADIUS * np.sum(position_diff) / 2

        # If the displacement is very small, we don't need to update anything
        if abs(rho) < 1e-2:
            return

        # Update theta (orientation) by calculating the difference in wheel positions, 
        # multiplying by the wheel radius, and dividing by the distance between the wheels
        dtheta = self.WHEEL_RADIUS * (position_diff[0] - position_diff[1]) / self.WHEEL_DISTANCE
        self.kin_pose[2] += -dtheta

        # Update the x and y position by adding the displacement times the cosine (for x) or sine (for y) of the current angle
        self.kin_pose[0] += rho * np.cos(self.kin_pose[2])
        self.kin_pose[1] += rho * np.sin(self.kin_pose[2])

        # plot odometry
        # Add positions to the list
        self.kin_odom_y.append(self.kin_pose[1])
        self.kin_odom_x.append(self.kin_pose[0])
        # Add a point at the current x, y position
        self.kin_odom_plot.set_xdata(self.kin_odom_y)
        self.kin_odom_plot.set_ydata(self.kin_odom_x)
        # Redraw the plot
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.fig.canvas.draw()
        plt.pause(0.001)

        # Update joint_states with the current message for the next time this function is called
        self.joint_states = msg

    def velocity_model(self, msg: Twist):
        if self.last_time is None:
            self.last_time = self.get_clock().now().nanoseconds
            return
        
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.last_time) / 1e9

        # Compute linear and angular velocities
        v = msg.linear.x
        w = msg.angular.z

        # Update x and y (position)
        self.vel_pose[0] += v * math.cos(self.vel_pose[2]) * dt
        self.vel_pose[1] += v * math.sin(self.vel_pose[2]) * dt
        # Update theta (orientation)
        self.vel_pose[2] += w * dt

        # Save the calculated odometry
        self.vel_odom_y.append(self.vel_pose[1])
        self.vel_odom_x.append(self.vel_pose[0])

        # Update plot data
        self.vel_odom_plot.set_xdata(self.vel_odom_y)
        self.vel_odom_plot.set_ydata(self.vel_odom_x)

        # Redraw the plot
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.fig.canvas.draw()
        plt.pause(0.001)

        # Save current time
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)

    node = VelocityOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
