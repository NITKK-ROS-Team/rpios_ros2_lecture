# RaspberryPi向けGPIO操作ライブラリ
import gpiozero
# ROS2ライブラリ
import rclpy
from rclpy.node import Node
# ROS2サービス型 SG90型
from servo_msgs.srv import SG90

# ノードクラスを継承して作成
class ServoExampleService(Node):
    def __init__(self):
        # rclpy.node.Nodeクラスの初期化。ノード名をservo_example_serviceに設定
        super().__init__('servo_example_service')

        # servo_pinパラメータを宣言し、17をデフォルト値として設定
        self.declare_parameter('servo_pin', 17)

        # servo_pinパラメータの値を取得してservoに設定
        self.servo = gpiozero.PWMOutputDevice(
            pin=self.get_parameter('servo_pin').value,
            frequency=50
        )

        # servoサービスを受信するためのサービスを作成
        self.srv = self.create_service(SG90, 'servo', self.servo_callback)

    # servoサービスを受信したときに実行される関数
    def servo_callback(self, request, response):
        self.get_logger().info('angle: %d' % request.angle)
        # duty_cycleを計算
        duty_cycle = (request.angle / 180) * 0.1 + 0.05
        # duty_cycleをservoに設定
        self.servo.value = duty_cycle
        # response.resultにTrueを設定
        response.result = True
        # responseを返す（Clientに送信）
        return response

# メイン関数
def main(args=None):
    # rclpyライブラリの初期化
    rclpy.init(args=args)
    # ServoExampleServiceクラスのインスタンスを作成
    servo_example_service = ServoExampleService()
    # ノードを実行 （Ctrl+Cで終了）
    rclpy.spin(servo_example_service)

    # ノードの終了処理
    servo_example_service.destroy_node()
    # rclpyライブラリの終了処理
    rclpy.shutdown()

if __name__ == '__main__':
    main()