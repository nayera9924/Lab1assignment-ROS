import rclpy
from rclpy.node import Node
from interfaces_pkg.srv import SetSpeedLimits

class SpeedControlNode(Node):
    def __init__(self):
        super().__init__('speed_control_node')

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        self.speed_service = self.create_service(
            SetSpeedLimits,
            'set_speed_limits',
            self.set_speed_callback
        )
        
        self.get_logger().info('Speed Control Node has been started with parameters and service.')

    def set_speed_callback(self, request, response):
        if request.max_linear_speed <= 0.0 or request.max_angular_speed <= 0.0:
            response.success = False
            response.message = "Failed: Speeds must be greater than zero!"
            self.get_logger().warn(response.message)
            return response
        
        self.set_parameters([
            rclpy.parameter.Parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE, request.max_linear_speed),
            rclpy.parameter.Parameter('max_angular_speed', rclpy.Parameter.Type.DOUBLE, request.max_angular_speed)
        ])

        self.max_linear_speed = request.max_linear_speed
        self.max_angular_speed = request.max_angular_speed
        
        response.success = True
        response.message = "Speed limits updated successfully!"
    
def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = SpeedControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()