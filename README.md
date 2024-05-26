# rpios_ros2_lecture
RaspberryPi OS Bookworm &amp; ROS 2 lecture

### create servo_example package

```bash
ros2 pkg create --build-type ament_python --dependencies rclpy servo_msgs --description "servo example" --package-format 3 --node-name servo_example_service --license Apache-2.0  servo_example
```

<br>

## 使用するキット

- SG90：[秋月電子](https://akizukidenshi.com/catalog/g/g108761/)
- フォトインタラプタ（例：OP90° 赤外線反射ユニット：[マルツ](https://www.marutsu.co.jp/pc/i/1634665/)）
- 固定用パーツ：[STLファイル](stl/servo_jig.stl)

<img width="624" alt="servo_base" src="https://github.com/NITKK-ROS-Team/rpios_ros2_lecture/assets/67567093/7c31096a-9944-4762-8d10-b6ea2980b665">


<br>

## 例

### ビルド

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/NITKK-ROS-Team/rpios_ros2_lecture.git
cd ~/ros2_ws

colcon build --symlink-install
```

<br>

### Publisher（送信のみ）

フォトインタラプタの状態を読み取り、その値をトピックとして送信する。

```bash
source ~/ros2_ws/install/setup.bash
ros2 run servo_example input_example_pub
```

受信する場合は、以下のコマンドを実行する。

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /input
```

<br>

### Subscriber（受信のみ）

トピックから受信した値をサーボモータに出力する。

```bash
source ~/ros2_ws/install/setup.bash
ros2 run servo_example servo_example_sub
```

送信する場合は、以下のコマンドを実行する。

```bash
# 0度
ros2 topic pub /servo std_msgs/msg/Int16 data:\ 0
# 90度
ros2 topic pub /servo std_msgs/msg/Int16 data:\ 90
```

<br>

### Service（送受信）

トピックから受信した値をサーボモータに出力する。送信されるまで待機し、即座に結果が返される。

```bash
source ~/ros2_ws/install/setup.bash
ros2 run servo_example servo_example_service
```

送信する場合は、以下のコマンドを実行する。

```bash
# 0度
ros2 service call /servo servo_msgs/srv/SG90 angle:\ 0
# 90度
ros2 service call /servo servo_msgs/srv/SG90 angle:\ 90
```

<br>

### Action（送受信）

トピックから受信した値をサーボモータに出力する。送信されるまで待機し、結果が返されるまで待機する。

```bash
source ~/ros2_ws/install/setup.bash
ros2 run servo_example servo_example_action
```

送信する場合は、以下のコマンドを実行する。

```bash
# 0度：フォトインタラプタの値が1になるまで待機
ros2 action send_goal /servo servo_msgs/action/SetAngle "{angle: 0}" --feedback

# 90度：即座に返答される（仮実装）
ros2 action send_goal /servo servo_msgs/action/SetAngle "{angle: 90}" --feedback
```
