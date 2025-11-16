from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    esp_port = LaunchConfiguration('esp_port')
    esp_baud = LaunchConfiguration('esp_baud')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('esp_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),

        Node(
            package='ros2_esp_bridge',
            executable='esp_serial_bridge',
            name='esp_serial_bridge',
            output='screen',
            parameters=[
                {'port': esp_port},
                {'baud': esp_baud},
            ],
        ),

        # This assumes you have a LiDAR driver package installed (e.g., sllidar_ros2)
        # If you don't, install it on the Pi or replace this node with your LiDAR driver.
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[
                {'serial_port': lidar_port},
                {'serial_baudrate': 115200},  # A1 default
                {'frame_id': 'laser_frame'},
                {'inverted': False},
                {'angle_compensate': True},
            ],
        ),
    ])
