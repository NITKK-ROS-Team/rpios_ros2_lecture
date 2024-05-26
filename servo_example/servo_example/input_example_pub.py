import gpiozero
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class InputExamplePub(Node):
    def __init__(self):
        super().__init__('input_example_pub')
        self.declare_parameter('input_pin', 4)
        self.input_pin = gpiozero.DigitalInputDevice(
            pin=self.get_parameter('input_pin').value,
            pull_up=False
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pub = self.create_publisher(Bool, 'input', 10)

    def timer_callback(self):
        msg = Bool()
        msg.data = bool(self.input_pin.value)
        self.get_logger().info('input: %d' % msg.data)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    input_example_pub = InputExamplePub()
    rclpy.spin(input_example_pub)
    input_example_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()