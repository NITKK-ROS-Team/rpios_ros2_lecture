# RaspberryPi向けGPIO操作ライブラリ
import gpiozero
# ROS2ライブラリ
import rclpy
from rclpy.node import Node
# ROS2メッセージ型 Int16型
from std_msgs.msg import Int16

# ノードクラスを継承して作成
class ServoExampleSub(Node):
    def __init__(self):
        # rclpy.node.Nodeクラスの初期化。ノード名をservo_example_subに設定
        super().__init__('servo_example_sub')

        # servo_pinパラメータを宣言し、17をデフォルト値として設定
        self.declare_parameter('servo_pin', 17)

        # servo_pinパラメータの値を取得してservoに設定
        self.servo = gpiozero.PWMOutputDevice(
            pin=self.get_parameter('servo_pin').value,
            frequency=50
        )

        # servoトピックを受信するためのサブスクライバーを作成
        # 10はキューサイズ
        self.sub = self.create_subscription(
            Int16,
            'servo',
            self.sub_callback,
            10
        )

    # servoトピックを受信したときに実行される関数
    def sub_callback(self, msg):
        self.get_logger().info('angle: %d' % msg.data)
        # duty_cycleを計算
        duty_cycle = (msg.data / 180) * 0.1 + 0.05
        # duty_cycleをservoに設定
        self.servo.value = duty_cycle

# メイン関数
def main(args=None):
    # rclpyライブラリの初期化
    rclpy.init(args=args)
    # ServoExampleSubクラスのインスタンスを作成
    servo_example_sub = ServoExampleSub()
    # ノードを実行 （Ctrl+Cで終了）
    rclpy.spin(servo_example_sub)

    # ノードの終了処理
    servo_example_sub.destroy_node()
    # rclpyライブラリの終了処理
    rclpy.shutdown()

if __name__ == '__main__':
    main()