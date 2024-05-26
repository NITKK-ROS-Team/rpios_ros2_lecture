# rpios_ros2_lecture
RaspberryPi OS Bookworm &amp; ROS 2 lecture

### create servo_example package

```bash
ros2 pkg create --build-type ament_python --dependencies rclpy servo_msgs --description "servo example" --package-format 3 --node-name servo_example_service --license Apache-2.0  servo_example
```