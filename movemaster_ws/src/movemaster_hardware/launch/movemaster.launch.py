"""Arranca ros2_control con MovemasterHardware (o mock) y los controladores.

    ros2 launch movemaster_hardware movemaster.launch.py
    ros2 launch movemaster_hardware movemaster.launch.py use_mock_hardware:=true
    ros2 launch movemaster_hardware movemaster.launch.py controller:=forward_position_controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_interface = LaunchConfiguration('can_interface')
    controller = LaunchConfiguration('controller')

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare('movemaster_hardware'),
                                  'description', 'urdf', 'movemaster.urdf.xacro']),
            ' use_mock_hardware:=', use_mock_hardware,
            ' can_interface:=', can_interface,
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare('movemaster_hardware'), 'config', 'movemaster_controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='true = mock_components/GenericSystem (sin CAN)'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('controller', default_value='joint_trajectory_controller',
                              description='joint_trajectory_controller o forward_position_controller'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers],
            remappings=[('~/robot_description', '/robot_description')],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--controller-manager', '/controller_manager'],
        ),
    ])
