#!/usr/bin/env python3
import threading
import time
import serial
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int8MultiArray, String
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

class ESPSerialBridge(Node):
    def __init__(self):
        super().__init__('esp_serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('wheel_radius', 0.035)  # meters
        self.declare_parameter('wheelbase', 0.24)      # meters  
        self.declare_parameter('ticks_per_rev', 231)   # encoder ticks per wheel revolution
        
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().integer_value
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = self.get_clock().now()
        self.first_reading = True

        self.cb_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_enc = self.create_publisher(Int32MultiArray, 'esp/encoder_counts', qos)
        self.pub_line = self.create_publisher(Int8MultiArray, 'esp/line_sensors', qos)
        self.pub_raw = self.create_publisher(String, 'esp/raw', qos)
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos)
        
        # TF broadcaster for odometry
        self.tf_broadcaster = TransformBroadcaster(self)

        self.srv_reset = self.create_service(Empty, 'esp/reset_encoders', self._handle_reset, callback_group=self.cb_group)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'Opened serial {self.port} at {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            raise

        self._stop = threading.Event()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

        self.timer = self.create_timer(1.0, self._periodic_ping)

    def _periodic_ping(self):
        try:
            self.ser.write(b'GET\n')
        except Exception as e:
            self.get_logger().warn(f'Write failed: {e}')

    def _handle_reset(self, request, response):
        try:
            self.ser.write(b'R\n')
            # Reset odometry state
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.first_reading = True
            self.get_logger().info('Reset encoders and odometry')
        except Exception as e:
            self.get_logger().warn(f'Write failed: {e}')
        return response

    def _reader_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                data = self.ser.read(128)
                if data:
                    buf.extend(data)
                    while b'\n' in buf:
                        line, _, rest = buf.partition(b'\n')
                        buf = bytearray(rest)
                        self._handle_line(line.decode(errors='ignore').strip())
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().warn(f'Read failed: {e}')
                time.sleep(0.5)

    def _handle_line(self, line: str):
        # Publish raw line for debugging
        self.pub_raw.publish(String(data=line))
        if not line:
            return
        # Telemetry: T,<ms>,<L>,<R>,<s1>,<s2>,<s3>,<s4>,<s5>
        if line.startswith('T,'):
            parts = line.split(',')
            if len(parts) == 9:
                try:
                    # ms = int(parts[1])  # unused here
                    L = int(parts[2])
                    R = int(parts[3])
                    s_vals = [int(x) for x in parts[4:9]]
                    self._pub_encoders(L, R)
                    self._pub_line_sensors(s_vals)
                except ValueError:
                    pass
        # Ack or other lines are ignored

    def _pub_encoders(self, L: int, R: int):
        msg = Int32MultiArray()
        msg.data = [L, R]
        self.pub_enc.publish(msg)
        
        # Calculate and publish odometry
        self._update_odometry(L, R)

    def _pub_line_sensors(self, s_vals):
        msg = Int8MultiArray()
        msg.data = s_vals
        self.pub_line.publish(msg)
    
    def _update_odometry(self, left_ticks: int, right_ticks: int):
        current_time = self.get_clock().now()
        
        if self.first_reading:
            # Initialize on first reading
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time = current_time
            self.first_reading = False
            return
        
        # Calculate change in encoder ticks
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks
        
        # Convert ticks to distance (meters)
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        left_distance = delta_left * meters_per_tick
        right_distance = delta_right * meters_per_tick
        
        # Calculate robot motion
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheelbase
        
        # Update robot pose
        delta_x = distance * math.cos(self.theta + delta_theta / 2.0)
        delta_y = distance * math.sin(self.theta + delta_theta / 2.0)
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi
        
        # Calculate velocities
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0:
            linear_vel = distance / dt
            angular_vel = delta_theta / dt
        else:
            linear_vel = 0.0
            angular_vel = 0.0
        
        # Publish odometry message
        self._publish_odometry(current_time, linear_vel, angular_vel)
        
        # Update last values
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time
    
    def _publish_odometry(self, current_time, linear_vel: float, angular_vel: float):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.frame_id
        
        # Position
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        
        # Orientation (quaternion from yaw)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        # Velocity
        odom.twist.twist.linear = Vector3(x=linear_vel, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=angular_vel)
        
        # Covariance (simple diagonal values - can be tuned)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y  
        odom.pose.covariance[35] = 0.02  # theta
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.02 # vtheta
        
        self.pub_odom.publish(odom)
        
        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = self.frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self._stop.set()
        try:
            if hasattr(self, 'ser') and self.ser:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESPSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
