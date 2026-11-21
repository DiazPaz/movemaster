"""Vista previa de la cinemática del MoveMaster, sin hardware ni controladores.

    ros2 launch movemaster_description display.launch.py [gui:=false] [rviz:=false]

Arranca:
  robot_state_publisher      publica /robot_description y el TF de la cadena DH.
  joint_state_publisher_gui  un slider por eje; arranca en q = 0, Home.
                             Con gui:=false, joint_state_publisher deja el brazo en Home.
  rviz2                      el modelo, el TF y tool0.

El URDF se genera con preview:=true: mueve los cinco ejes aunque joints.json todavía no
los tenga a todos, y no incluye el bloque ros2_control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    share = FindPackageShare('movemaster_description')
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='true: sliders de joint_state_publisher_gui; '
                        'false: el brazo queda en Home.'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='Abrir RViz con rviz/display.rviz.'),
        DeclareLaunchArgument(
            'joint_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('movemaster_hardware'), 'config', 'joints.json']),
            description='joints.json: límites de los ejes que ya tienen SPARK MAX.'),
        DeclareLaunchArgument(
            'dh_config', default_value=PathJoinSubstitution([share, 'config', 'dh.yaml']),
            description='Tabla DH del brazo.'),
        OpaqueFunction(function=_launch_setup),
        Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'display.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    description_file = PathJoinSubstitution(
        [FindPackageShare('movemaster_description'), 'urdf', 'movemaster.urdf.xacro']
    ).perform(context)
    robot_description = xacro.process_file(description_file, mappings={
        'joint_config': arg('joint_config'),
        'dh_config': arg('dh_config'),
        'preview': 'true',
    }).toxml()
    # joint_state_publisher arranca cada eje en 0 (Home) si está dentro de sus límites.
    publisher = 'joint_state_publisher_gui' if arg('gui') == 'true' else 'joint_state_publisher'

    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='both', parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package=publisher, executable=publisher, output='both',
        ),
    ]
