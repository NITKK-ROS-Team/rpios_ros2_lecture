# RaspberryPi向けGPIO操作ライブラリ
import gpiozero
# ROS2ライブラリ
import rclpy
from rclpy.node import Node
# ROS2メッセージ型 Bool型
from std_msgs.msg import Bool

# ノードクラスを継承して作成
class InputExamplePub(Node):
    def __init__(self):
        # rclpy.node.Nodeクラスの初期化。ノード名をinput_example_pubに設定
        super().__init__('input_example_pub')

        # input_pinパラメータを宣言し、4をデフォルト値として設定
        self.declare_parameter('input_pin', 4)

        # input_pinパラメータの値を取得してinput_pinに設定
        self.input_pin = gpiozero.DigitalInputDevice(
            pin=self.get_parameter('input_pin').value,
            pull_up=False
        )

        # 0.1秒ごとにtimer_callback関数を実行
        self.timer = self.create_timer(0.1, self.timer_callback)

        # inputトピックを送信するためのパブリッシャーを作成
        self.pub = self.create_publisher(Bool, 'input', 10)

    # self.timerによって実行される関数
    def timer_callback(self):
        # Bool型のメッセージを作成
        msg = Bool()
        # msg.dataにinput_pinの値を設定
        msg.data = bool(self.input_pin.value)
        self.get_logger().info('input: %d' % msg.data)

        # `input_pin`の値を`/input`トピックに送信
        self.pub.publish(msg)

# メイン関数
def main(args=None):
    # rclpyライブラリの初期化
    rclpy.init(args=args)
    # InputExamplePubクラスのインスタンスを作成
    input_example_pub = InputExamplePub()
    # ノードを実行 （Ctrl+Cで終了）
    rclpy.spin(input_example_pub)

    # ノードの終了処理
    input_example_pub.destroy_node()
    # rclpyライブラリの終了処理
    rclpy.shutdown()

if __name__ == '__main__':
    main()