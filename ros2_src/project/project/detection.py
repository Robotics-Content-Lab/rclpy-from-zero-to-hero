#! /usr/bin/env python3
from typing import List

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult # Handles responses to parameter change requests
from rclpy.parameter import Parameter # Handles parameters within a node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from example_interfaces.msg import Float32MultiArray, Int8, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2


class LineDetector(Node):
    def __init__(self):
        super().__init__('detection')
        self.image_width = None
        self.image_height = None
        self.img_threshold = None
        self.bridge = CvBridge()
        
        self.init_params()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.image_pub = self.create_publisher(Image, '/line_detection', 10)
        self.position_pub = self.create_publisher(Float32MultiArray, '/line_position', 10)
        self.direction_pub = self.create_publisher(Int8, '/direction', 10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.no_line_detected_pub = self.create_publisher(Bool, '/no_line_detected', qos)

    def init_params(self):
        self.direction_threshold = self.declare_parameter('direction_threshold', 10).value
        self.image_threshold = self.direction_threshold
        self.image_crop = self.declare_parameter('image_crop', 0.5).value
        self.show_images = self.declare_parameter('show_images', False).value
        self.show_annotations = self.declare_parameter('show_annotations', True).value

    def parameter_change_cb(self, params: List[Parameter]) -> SetParametersResult:
        result = SetParametersResult()
        result.successful = False
 
        for param in params:
            if param.name == 'direction_threshold' and param.type_ == Parameter.Type.INTEGER:
                if not param.value >= 0 and param.value <= 100:
                    result.reason = 'The value of direction_threshold must be between 0 and 100.'
                else:
                    self.direction_threshold = param.value
                    self.img_threshold = self.image_width * (self.direction_threshold / 100) if self.image_width is not None else 10
                    result.successful = True
            if param.name == 'show_images' and param.type_ == Parameter.Type.BOOL:
                self.show_images = param.value
                result.successful = True
            if param.name == 'show_annotations' and param.type_ == Parameter.Type.BOOL:
                self.show_annotations = param.value
                result.successful = True
            if param.name == 'image_crop' and param.type_ == Parameter.Type.DOUBLE:
                if not param.value > 0 and param.value <= 1:
                    result.reason = 'The value of image_crop must be between 0 and 1.'
                else:
                    self.image_crop = param.value
                    result.successful = True
 
        return result

    def listener_callback(self, data: Image):
        if self.image_width is None:
            self.image_width = data.width
            self.image_height = data.height
        self.img_threshold = self.image_width * (self.direction_threshold / 100)

        # Convert ROS Image message to OpenCV image
        default_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # Crop the image to the bottom 1/2
        cropped_frame = self.crop_image(default_frame, int(self.image_height * self.image_crop))

        # Segment yellow color
        yellow_segmented_image = self.segment_yellow(cropped_frame)

        # Get the mass centers of the yellow objects
        mass_centers, contours, bw_image = self.get_mass_centers(yellow_segmented_image)
        
        # Draw the mass centers and contours on the original image
        result_image = self.draw_image(default_frame, mass_centers, contours, cropped_frame, self.show_annotations)

        if self.get_parameter('show_images').value:
            cv2.imshow("default_frame", default_frame)
            cv2.imshow("yellow Segmented Image0", yellow_segmented_image)
            cv2.imshow("bw_image", bw_image)
            cv2.imshow("result_image", result_image)
            cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()
        if mass_centers:
            self.publish(mass_centers, result_image)
        else:
            self.publish_no_line_detected()
        
    @staticmethod
    def crop_image(image, height):
        """Crop the image to specified height starting from bottom

        Args:
            image: The image to crop
            height: The height to crop from bottom
        """
        return image[-height:, :]

    @staticmethod
    def segment_yellow( image):
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        lower_yellow = np.array([15, 50, 50])   # Lower bound of yellow color
        upper_yellow = np.array([35, 255, 255])  # Upper bound of yellow color

        # Create a binary mask
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Apply morphological operations to remove small noise
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        # Apply the mask to the original image
        yellow_segmented_image = cv2.bitwise_and(image, image, mask=yellow_mask)
        return yellow_segmented_image

    @ staticmethod
    def get_mass_centers(image):
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        (_, bw_image) = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # Calculate moments based on bw_image
        moments= cv2.moments(bw_image)
        canny = cv2.Canny(bw_image, 100, 200)
        contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mass_centers = []
        if moments:
            # Get the mass centers
            if moments['m00'] != 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                # mass_centers.append((cx, cy))
                # Reference the x position to the center of the width of the image
                x = int(cx - (bw_image.shape[1] / 2))
                # Reference the y position to the bottom of the image
                y = int(bw_image.shape[0] - cy)
                mass_centers.append((x, y))
        else:
            mass_centers = []

        return mass_centers, contours, bw_image

    @staticmethod
    def draw_image(image, mass_centers, contours, cropped_frame, show_annotations=True):
        result_image = image.copy()
        # Draw vertical line in the middle of the image
        cv2.line(result_image, (int(image.shape[1] / 2), 0), (int(image.shape[1] / 2), image.shape[0]), (0, 0, 255), 2)
        for i, center in enumerate(mass_centers):
            og_center = (int(center[0] + (cropped_frame.shape[1]/2)), center[1] + cropped_frame.shape[0])
            if show_annotations:
                cv2.putText(result_image, f"x:{center[0]}|y:{center[1]}", (og_center[0] - 40, og_center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.arrowedLine(result_image, (int(image.shape[1] / 2), og_center[1]), og_center, (0, 0, 255), 2)
                cv2.arrowedLine(result_image, (og_center[0], result_image.shape[0]), og_center, (0, 0, 255), 2)
            cv2.circle(result_image, og_center, 5, (0, 255, 0), -1)
        for contour in contours:
            contour = contour + np.array([0, cropped_frame.shape[0]])
            cv2.drawContours(result_image, [contour], 0, (0, 255, 0), 3)

        return result_image

    def publish(self, mass_centers, result_image):
        x_position = -1 * mass_centers[0][0]
        y_position = mass_centers[0][1]

        # Publish the mass centers
        position_msg = Float32MultiArray()
        position_msg.data = []
        position_msg.data.append(x_position)
        position_msg.data.append(y_position)
        self.position_pub.publish(position_msg)

        # Publish the direction
        direction = Int8()
        if len(mass_centers) == 0:
            direction.data = -1
        else:
            half_width = self.image_width / 2
            if x_position < half_width - self.img_threshold:
                direction.data = 0
            elif x_position > half_width + self.img_threshold:
                direction.data = 2
            else:
                direction.data = 1
        self.direction_pub.publish(direction)

        # Publish the image
        image_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
        self.image_pub.publish(image_msg)

    def publish_no_line_detected(self):
        self.no_line_detected_pub

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = LineDetector()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
