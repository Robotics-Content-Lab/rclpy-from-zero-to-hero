#! /usr/bin/env python3
from typing import List 

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor # Enables the description of parameters
from rcl_interfaces.msg import SetParametersResult # Handles responses to parameter change requests
from rclpy.parameter import Parameter # Handles parameters within a node
 
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
 
        scan_radius_descriptor = ParameterDescriptor(description='Radius of the scan area')
        scan_angle_descriptor = ParameterDescriptor(description='Angle of the scan area')
 
        # Declare parameters
        self.declare_parameter('scan_radius', 15.0, scan_radius_descriptor)
        self.declare_parameter('scan_angle', 360, scan_angle_descriptor)
        self.scan_radius = self.get_parameter('scan_radius').value
        self.scan_angle = self.get_parameter('scan_angle').value
 
        # Register a callback function that will be called whenever there is an attempt to
        # change one or more parameters of the node.
        self.add_on_set_parameters_callback(self.parameter_change_cb)
 
    def parameter_change_cb(self, params: List[Parameter]) -> SetParametersResult:
        """Gets called whenever there is an attempt to change one or more parameters.
        Args:
            params (List[Parameter]): A list of Parameter objects representing the parameters that are 
                being attempted to change.
        Returns:
            SetParametersResult: Object indicating whether the change was successful.
        """
        result = SetParametersResult()
 
        # Iterate over each parameter in this node
        for param in params:
            # Check the parameter's name and type
            if param.name == 'scan_radius' and param.type_ == Parameter.Type.DOUBLE:
                # This parameter has changed. Display the new value to the terminal.
                self.scan_radius = param.value
                self.get_logger().info('Parameter scan_radius has changed. The new value is: %f' % self.scan_radius)
                # The parameter change was successfully handled.
                result.successful = True
            if param.name == 'scan_angle' and param.type_ == Parameter.Type.INTEGER:
                if param.value < 0 or param.value > 360:
                    # The parameter change was not successful because the value is out of range.
                    result.successful = False
                    result.reason = 'The value of scan_angle must be between 0 and 360.'
                    self.get_logger().info('Parameter scan_angle change has failed: %s' % result.reason)
                else:
                    self.get_logger().info('Parameter scan_angle has changed. The new value is: %s' % param.value)
                    result.successful = True
 
        return result
 
def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()
    rclpy.spin(parameter_node)
    parameter_node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()