"""jog_node with config/jog.yaml.

    ros2 launch movemaster_teleop jog.launch.py [port:=/dev/ttyUSB0]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Serial port of the ESP32 pendant.'),
        Node(
            package='movemaster_teleop', executable='jog_node', output='both',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('movemaster_teleop'), 'config', 'jog.yaml']),
                {'port': LaunchConfiguration('port')},
            ],
        ),
    ])
