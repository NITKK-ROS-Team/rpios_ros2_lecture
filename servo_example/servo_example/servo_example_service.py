import gpiozero
import rclpy
from rclpy.node import Node
from servo_msgs.srv import SG90

class ServoExampleService(Node):
    def __init__(self):
        super().__init__('servo_example_service')
        self.declare_parameter('servo_pin', 17)
        self.servo = gpiozero.PWMOutputDevice(
            pin=self.get_parameter('servo_pin').value,
            frequency=50
        )
        self.srv = self.create_service(SG90, 'servo', self.servo_callback)

    def servo_callback(self, request, response):
        self.get_logger().info('angle: %d' % request.angle)
        duty_cycle = (request.angle / 180) * 0.1 + 0.05
        self.servo.value = duty_cycle
        response.result = True
        return response

def main(args=None):
    rclpy.init(args=args)
    servo_example_service = ServoExampleService()
    rclpy.spin(servo_example_service)
    servo_example_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()