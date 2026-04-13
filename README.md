# LTM2985 UART ROS 2 package

This workspace contains two ROS 2 packages:

- `ltm2985_msgs` - custom message definition for parsed UART frames
- `ltm2985_uart` - Python ROS 2 node that reads UART, parses one JSON object per line, and publishes:
  - `ltm2985/raw_json` (`std_msgs/msg/String`)
  - `ltm2985/measurement` (`ltm2985_msgs/msg/Measurement`)

The node is based on the Arduino code in `untitled.zip`, which prints frames like:

```json
{"channel":1,"type":"temperature_K","value":298.45,"raw_code":12345,"sensor_value":0.12,"fault":1,"valid":true}
```

## Build

```bash
cd <workspace>
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch ltm2985_uart ltm2985_uart.launch.py port:=/dev/ttyUSB0 baudrate:=230400
```

## Topics

```bash
ros2 topic echo /ltm2985/raw_json
ros2 topic echo /ltm2985/measurement
```
