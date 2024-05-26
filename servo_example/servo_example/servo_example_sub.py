import gpiozero
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class ServoExampleSub(Node):
    def __init__(self):
        super().__init__('servo_example_sub')
        self.declare_parameter('servo_pin', 17)
        self.servo = gpiozero.PWMOutputDevice(
            pin=self.get_parameter('servo_pin').value,
            frequency=50
        )
        self.sub = self.create_subscription(
            Int16,
            'servo',
            self.sub_callback,
            10
        )

    def sub_callback(self, msg):
        self.get_logger().info('angle: %d' % msg.data)
        duty_cycle = (msg.data / 180) * 0.1 + 0.05
        self.servo.value = duty_cycle

def main(args=None):
    rclpy.init(args=args)
    servo_example_sub = ServoExampleSub()
    rclpy.spin(servo_example_sub)
    servo_example_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()