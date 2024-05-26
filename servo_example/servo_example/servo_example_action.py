import gpiozero
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from servo_msgs.action import SetAngle
import time

class ServoExampleAction(Node):
    def __init__(self):
        super().__init__('servo_example_service')
        self.declare_parameter('servo_pin', 17)
        self.declare_parameter('interrupter_pin', 4)
        self.servo = gpiozero.PWMOutputDevice(
            pin=self.get_parameter('servo_pin').value,
            frequency=50
        )
        self.input_pin = gpiozero.DigitalInputDevice(pin=4, pull_up=False)
        self.act_srv = ActionServer(
            self,
            SetAngle,
            'servo',
            self.act_callback
        )

    def act_callback(self, goal_handle):
        self.get_logger().info('angle: %d' % goal_handle.request.angle)
        duty_cycle = (goal_handle.request.angle / 180) * 0.1 + 0.05
        self.servo.value = duty_cycle
        feedback_msg = SetAngle.Feedback()

        while True:
            # if angle is not 0, sleep 1s and break
            if goal_handle.request.angle != 0:
                time.sleep(1)
                break

            # if angle is 0, check input_pin is 1 or not
            if self.input_pin.value == 1:
                break
            feedback_msg.feedback = bool(self.input_pin.value)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal_handle.succeed()
        result = SetAngle.Result()
        result.result = True
        return result

def main(args=None):
    rclpy.init(args=args)
    servo_example_service = ServoExampleAction()
    rclpy.spin(servo_example_service)
    servo_example_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()