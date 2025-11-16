from setuptools import setup

package_name = 'ros2_esp_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/esp_with_lidar.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial', 'rclpy', 'tf2-ros-py', 'tf-transformations'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 serial bridge for ESP32 robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp_serial_bridge = ros2_esp_bridge.esp_serial_bridge:main',
            'rectangle_path = ros2_esp_bridge.rectangle_path:main',
        ],
    },
)
