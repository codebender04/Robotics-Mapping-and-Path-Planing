# ros2_esp_bridge

ROS 2 Humble serial bridge to an ESP32 robot over USB serial.

It talks a simple, line-oriented CSV protocol with the ESP32 firmware and exposes ROS topics and a service:

- Publishes encoder counts: `esp/encoder_counts` (std_msgs/Int32MultiArray) -> [left, right]
- Publishes line sensors: `esp/line_sensors` (std_msgs/Int8MultiArray) -> [s1..s5]
- Publishes raw lines (debug): `esp/raw` (std_msgs/String)
- Service to reset encoders: `esp/reset_encoders` (std_srvs/Empty)

## ESP32 serial protocol

- Outgoing telemetry (20 Hz):

```
T,<millis>,<Lcount>,<Rcount>,<s1>,<s2>,<s3>,<s4>,<s5>\n
```

- Incoming commands:

```
R       # reset encoders
PING    # respond with PONG
GET     # send one telemetry frame immediately
```

## Build and run (on Raspberry Pi 4 with ROS 2 Humble)

```bash
# Create a workspace (if you don't have one already)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy this package into the src/ folder, then:
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Run the bridge (adjust serial port if needed)
ros2 run ros2_esp_bridge esp_serial_bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```

Typical ports:
- `/dev/ttyUSB0` or `/dev/ttyACM0` on the Pi. Check with `ls /dev/tty*` after plugging in the ESP32.

## Notes
- Requires `pyserial`: installed via `pip3 install pyserial` or through rosdep.
- If you get permission errors opening the serial port, add your user to the `dialout` group and re-login:

```bash
sudo usermod -a -G dialout $USER
```

- The RPLidar A1 continues to run as its own ROS 2 driver; this bridge is independent and can be composed with it in a launch file later.

## RPLidar A1 (LiDAR) integration

This package does not include a LiDAR driver. Install one on the Pi (e.g., `sllidar_ros2`) and use the provided launch file to run both the ESP bridge and LiDAR together.

Install options on the Pi:
- Prefer binary if available for Humble, or build from source:
	- Binary (if available):
		- `sudo apt search sllidar_ros2` (check availability)
		- `sudo apt install ros-humble-sllidar-ros2`
	- From source:
		- `cd ~/ros2_ws/src && git clone https://github.com/Slamtec/sllidar_ros2.git`
		- `cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build`

Launch both:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ros2_esp_bridge esp_with_lidar.launch.py esp_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

Adjust ports as needed. The launch assumes the A1’s default baud (115200) and publishes LaserScan on the LiDAR driver’s standard topic (e.g., `/scan`).
