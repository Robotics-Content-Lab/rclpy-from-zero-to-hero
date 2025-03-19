import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


class OdomComparison(Node):
    def __init__(self):
        super().__init__('odom_comparison')
        client_cb_group = ReentrantCallbackGroup()

        self.sub_odom_ekf = self.create_subscription(
            Odometry,
            'odom_ekf',
            self.odom_ekf_cb,
            10,
            callback_group=client_cb_group
        )
        self.sub_odom_kf = self.create_subscription(
            Odometry,
            'odom_kf',
            self.odom_kf_cb,
            10,
            callback_group=client_cb_group
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_cb,
            10,
            callback_group=client_cb_group
        )

        self.init_plot()

    def init_plot(self):
        plt.ion()
        self.fig, self.ax1 = plt.subplots()
        self.ax1.set_title("Robot Odometry")
        self.ax1.set_xlabel("x [m]")
        self.ax1.set_ylabel("y [m]")
        
        self.kf_odom_x = []
        self.kf_odom_y = []
        self.ekf_odom_x = []
        self.ekf_odom_y = []
        self.gt_odom_x = []
        self.gt_odom_y = []

        self.gt_odom_plot, = self.ax1.plot([], [], 'k*', label='GT Odometry', alpha=0.5)
        self.ekf_odom_plot, = self.ax1.plot([], [], 'b-', label='EKF Odometry', linewidth=3.0)
        self.kf_odom_plot, = self.ax1.plot([], [], 'r-', label='KF Odometry', linewidth=3.0)

        self.covariance_scale_factor = 0.5
        self.covariance_ellipse_ekf = Ellipse((0, 0), 0, 0, angle=0, edgecolor='b', facecolor='none', label='EKF Covariance')
        self.ax1.add_patch(self.covariance_ellipse_ekf)
        self.covariance_ellipse_kf = Ellipse((0, 0), 0, 0, angle=0, edgecolor='r', facecolor='none', label='KF Covariance')
        self.ax1.add_patch(self.covariance_ellipse_kf)

        self.ax1.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=3)


    def update_plot_limits(self):
        filter_x = self.ekf_odom_x + self.kf_odom_x
        filter_y = self.ekf_odom_y + self.kf_odom_y

        if not filter_x and not self.gt_odom_x:
            return

        if filter_x:
            x_lim_min = min(filter_x)
            x_lim_max = max(filter_x)
        else:
            x_lim_min = x_lim_max = 0

        if filter_y:
            y_lim_min = min(filter_y)
            y_lim_max = max(filter_y)
        else:
            y_lim_min = y_lim_max = 0

        if self.gt_odom_x:
            x_lim_min = min(x_lim_min, min(self.gt_odom_x))
            x_lim_max = max(x_lim_max, max(self.gt_odom_x))

        if self.gt_odom_y:
            y_lim_min = min(y_lim_min, min(self.gt_odom_y))
            y_lim_max = max(y_lim_max, max(self.gt_odom_y))

        # Calculate the new limits for the plot
        new_x_min = min(-1.0, x_lim_min - 0.2)
        new_x_max = max(1.0, x_lim_max + 0.2)
        new_y_min = min(-1.0, y_lim_min - 0.2)
        new_y_max = max(1.0, y_lim_max + 0.2)

        # Set the new limits for the plot
        self.ax1.set_xlim(new_x_min, new_x_max)
        self.ax1.set_ylim(new_y_min, new_y_max)

        self.fig.tight_layout()


    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.gt_odom_x.append(x)
        self.gt_odom_y.append(y)

        self.gt_odom_plot.set_xdata(self.gt_odom_x)
        self.gt_odom_plot.set_ydata(self.gt_odom_y)

        self.ax1.relim()
        self.update_plot_limits()
        self.fig.canvas.draw()
        plt.pause(0.001)

    def odom_kf_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.kf_odom_x.append(x)
        self.kf_odom_y.append(y)

        self.kf_odom_plot.set_xdata(self.kf_odom_x)
        self.kf_odom_plot.set_ydata(self.kf_odom_y)

        self.update_covariance_ellipse(msg.pose.covariance, x, y, self.covariance_ellipse_kf)

        self.ax1.relim()
        self.update_plot_limits()
        self.fig.canvas.draw()
        plt.pause(0.001)

    def odom_ekf_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.ekf_odom_x.append(x)
        self.ekf_odom_y.append(y)

        self.ekf_odom_plot.set_xdata(self.ekf_odom_x)
        self.ekf_odom_plot.set_ydata(self.ekf_odom_y)

        self.update_covariance_ellipse(msg.pose.covariance, x, y, self.covariance_ellipse_ekf)

        self.ax1.relim()
        self.update_plot_limits()
        self.fig.canvas.draw()
        plt.pause(0.001)

    def update_covariance_ellipse(self, covariance, x, y, covariance_ellipse):
        cov_matrix = np.array(covariance).reshape(6, 6)
        cov_xy = cov_matrix[:2, :2]

        eigenvalues, eigenvectors = np.linalg.eig(cov_xy)
        largest_eigenval_index = np.argmax(eigenvalues)
        largest_eigenvector = eigenvectors[:, largest_eigenval_index]
        smallest_eigenval_index = np.argmin(eigenvalues)
        smallest_eigenvector = eigenvectors[:, smallest_eigenval_index]

        angle = np.arctan2(largest_eigenvector[1], largest_eigenvector[0])
        width, height = self.covariance_scale_factor * 2 * np.sqrt(eigenvalues)

        covariance_ellipse.set_width(width)
        covariance_ellipse.set_height(height)
        covariance_ellipse.angle = np.degrees(angle)
        covariance_ellipse.set_center((x, y))

        # Make sure the ellipse is updated in the plot
        covariance_ellipse.set_visible(True)
        self.ax1.add_patch(covariance_ellipse)  # Ensure the ellipse is added to the plot

        # Redraw the plot
        self.fig.canvas.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = OdomComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
