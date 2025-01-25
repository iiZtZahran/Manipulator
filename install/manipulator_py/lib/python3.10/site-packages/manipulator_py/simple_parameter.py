import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")

        # Declare parameters with default values
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "zahran")

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params):
        result = SetParametersResult()
        result.successful = False  # Default to unsuccessful

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("simple_int_param new value is %d" % param.value)
                result.successful = True
            elif param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("simple_string_param new value is %s" % param.value)
                result.successful = True

        return result

def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
