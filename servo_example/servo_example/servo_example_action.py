# RaspberryPi向けGPIO操作ライブラリ
import gpiozero
# ROS2ライブラリ
import rclpy
# ROS2アクションライブラリ
from rclpy.action import ActionServer
from rclpy.node import Node
# ROS2アクションメッセージ型 SetAngle型
from servo_msgs.action import SetAngle
# timeライブラリ
import time

# ノードクラスを継承して作成
class ServoExampleAction(Node):
    def __init__(self):
        # rclpy.node.Nodeクラスの初期化。ノード名をservo_example_serviceに設定
        super().__init__('servo_example_service')

        # servo_pinパラメータを宣言し、17をデフォルト値として設定
        self.declare_parameter('servo_pin', 17)
        # interrupter_pinパラメータを宣言し、4をデフォルト値として設定
        self.declare_parameter('interrupter_pin', 4)

        # servo_pinパラメータの値を取得してservoに設定
        self.servo = gpiozero.PWMOutputDevice(
            pin=self.get_parameter('servo_pin').value,
            frequency=50
        )

        # interrupter_pinパラメータの値を取得してinput_pinに設定
        self.input_pin = gpiozero.DigitalInputDevice(pin=4, pull_up=False)

        # servoアクションを受信するためのアクションサーバーを作成
        # act_callback関数がアクションを受信したときに実行される
        self.act_srv = ActionServer(
            self,
            SetAngle,
            'servo',
            self.act_callback
        )

    # act_callback関数
    def act_callback(self, goal_handle):
        self.get_logger().info('angle: %d' % goal_handle.request.angle)
        # duty_cycleを計算
        duty_cycle = (goal_handle.request.angle / 180) * 0.1 + 0.05
        # duty_cycleをservoに設定
        self.servo.value = duty_cycle
        # SetAngle型のFeedbackメッセージを作成
        feedback_msg = SetAngle.Feedback()

        while True:
            # request.angleが0でない場合は1秒待機して終了
            if goal_handle.request.angle != 0:
                time.sleep(1)
                break

            # input_pinの値を取得・1の場合は終了
            if self.input_pin.value == 1:
                break
            # feedbackメッセージにinput_pinの値を設定
            feedback_msg.feedback = bool(self.input_pin.value)
            # feedbackメッセージを送信
            goal_handle.publish_feedback(feedback_msg)
            # 0.1秒待機
            time.sleep(0.1)

        # アクションを終了 (成功)
        goal_handle.succeed()
        # SetAngle型のResultメッセージを作成
        result = SetAngle.Result()
        # result.resultにTrueを設定
        result.result = True
        # resultを返す（Clientに送信）
        return result

# メイン関数
def main(args=None):
    # rclpyライブラリの初期化
    rclpy.init(args=args)
    # ServoExampleActionクラスのインスタンスを作成
    servo_example_service = ServoExampleAction()
    # ノードを実行 （Ctrl+Cで終了）
    rclpy.spin(servo_example_service)

    # ノードの終了処理
    servo_example_service.destroy_node()
    # rclpyライブラリの終了処理
    rclpy.shutdown()

if __name__ == '__main__':
    main()