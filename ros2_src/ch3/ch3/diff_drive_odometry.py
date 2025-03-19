import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        # Initialize pose as a numpy array [x, y, theta]
        # x and y represent the robot's position in the world frame
        # theta represents the robot's orientation in the world frame
        self.pose = np.array([0.0, 0.0, 0.0])

        # Initialize joint_states to None. This will be updated with the latest joint states from the robot.
        self.joint_states = None

        # Set constants
        # WHEEL_RADIUS: The radius of the robot's wheels
        # WHEEL_DISTANCE: The distance between the robot's wheels
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_DISTANCE = 0.160

        # Store past x/y positions for plotting
        self.x_positions = []
        self.y_positions = []

        # Subscribe to joint_state topic
        # This sets up a subscription to the 'joint_states' topic. 
        # Whenever a new message is published on this topic, the 'kinematic_model' method will be called with the message as its argument.
        self.subscription2 = self.create_subscription(
            JointState,
            'joint_states',
            self.kinematic_model,
            10
        )

        # Initialize the plot for visualizing the robot's odometry
        self.init_plot()

    def init_plot(self):
        # Set up pyplot for visualization
        # plt.ion() allows for interactive plots
        plt.ion()

        # Create a new figure and axes for the plot
        self.fig, self.ax1, = plt.subplots(1, 1)

        # Set the title and labels for the plot
        self.ax1.set_title("Robot Odometry")
        self.ax1.set_xlabel("y [m]")
        self.ax1.set_ylabel("x [m]")

        # Set the aspect ratio of the plot to be equal. This means that units on the x and y axes are the same size.
        self.ax1.set_aspect('equal')

    def update_plot_limits(self):
        # Calculate the new limits for the plot
        new_x_min = min(self.x_positions) - 0.1
        new_x_max = max(self.x_positions) + 0.1
        new_y_min = min(self.y_positions) - 0.1
        new_y_max = max(self.y_positions) + 0.1

        self.ax1.set_xlim(new_x_min, new_x_max)
        self.ax1.set_ylim(new_y_min, new_y_max)


    def kinematic_model(self, msg: JointState):
        # This method is called whenever a new JointState message is published on the 'joint_states' topic

        # If this is the first JointState message we've received, just store it and return
        if self.joint_states is None:
            self.joint_states = msg
            return

        # Calculate rho, the average displacement of the wheels
        # This is done by taking the difference in position of each wheel (current - previous), 
        # multiplying by the wheel radius, and averaging the two
        position_diff = np.array(msg.position) - np.array(self.joint_states.position)
        rho = self.WHEEL_RADIUS * np.sum(position_diff) / 2

        # If the displacement is very small, we don't need to update anything
        if abs(rho) < 1e-2:
            return

        # Update theta (orientation) by calculating the difference in wheel positions, 
        # multiplying by the wheel radius, and dividing by the distance between the wheels
        dtheta = self.WHEEL_RADIUS * (position_diff[0] - position_diff[1]) / self.WHEEL_DISTANCE
        self.pose[2] += dtheta

        # Update the x and y position by adding the displacement times the cosine (for x) or sine (for y) of the current angle
        self.pose[0] += rho * np.cos(self.pose[2])
        self.pose[1] += rho * np.sin(self.pose[2])

        # plot odometry
        # Add positions to the list
        self.x_positions.append(self.pose[1])
        self.y_positions.append(self.pose[0])
        # Add a point at the current x, y position
        self.ax1.scatter(self.pose[1], self.pose[0], color='r')
        self.update_plot_limits()
        # Draw and show plot
        self.fig.canvas.draw()
        plt.show()
        plt.pause(0.001)

        # Update joint_states with the current message for the next time this function is called
        self.joint_states = msg

def main(args=None):
    rclpy.init(args=args)

    node = Odometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()