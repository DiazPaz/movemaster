"""MoveMaster con un solo comando.

    ros2 launch movemaster_bringup bringup.launch.py [pendant:=/dev/ttyUSB0] [rviz:=true]
        [use_mock_hardware:=true] [joint_config:=...] [can_interface:=can0]

Incluye:
  movemaster_control  controller_manager con MovemasterHardware, joint_state_broadcaster,
                      joint_trajectory_controller activo y jog_controller inactivo.
  movemaster_teleop   jog_node, solo con pendant:=<puerto>. El switch MANUAL del pendant
                      elige el controlador: MANUAL, jog; AUTO, trayectorias.
  rviz2               solo con rviz:=true; necesita pantalla.

El servicio systemd/movemaster.service lo arranca al encender la Raspberry Pi.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotEqualsSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joint_config = LaunchConfiguration('joint_config')
    pendant = LaunchConfiguration('pendant')
    return LaunchDescription([
        DeclareLaunchArgument(
            'pendant', default_value='',
            description='Puerto serie del teach pendant (ESP32), por ejemplo /dev/ttyUSB0 o '
                        '/dev/serial/by-id/...; vacío: sin jog.'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Abrir RViz con el modelo del brazo; necesita pantalla.'),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false', choices=['true', 'false'],
            description='true: hardware simulado, sin CAN ni motores.'),
        DeclareLaunchArgument(
            'joint_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('movemaster_hardware'), 'config', 'joints.json']),
            description='joints.json: articulaciones, calibración y period_s.'),
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='Interfaz SocketCAN de los SPARK MAX.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('movemaster_control'), 'launch',
                 'movemaster_control.launch.py'])),
            launch_arguments={
                'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
                'joint_config': joint_config,
                'can_interface': LaunchConfiguration('can_interface'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('movemaster_teleop'), 'launch', 'jog.launch.py'])),
            launch_arguments={'port': pendant, 'joint_config': joint_config}.items(),
            condition=IfCondition(NotEqualsSubstitution(pendant, ''))),
        Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('movemaster_description'), 'rviz', 'display.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
