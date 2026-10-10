"""Nodo controller_manager de MoveMaster (ros2_control, ROS 2 Jazzy).

    ros2 launch movemaster_control movemaster_control.launch.py [use_mock_hardware:=true]

Arranca:
  robot_state_publisher  publica /robot_description (movemaster_description), de donde
                         el controller_manager carga el hardware, y el TF del robot.
  controller_manager     ros2_control_node con MovemasterHardware o hardware simulado.
  spawner                carga y activa joint_state_broadcaster y joint_trajectory_controller.

joints.json es la única fuente de las articulaciones: de él salen los ejes móviles del
URDF y su bloque ros2_control, los joints del JointTrajectoryController y la frecuencia
del lazo (1 / period_s). La geometría sale de la tabla DH de movemaster_description.
"""

import json
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import yaml

# Mismos valores que DriverConfig: periodo por defecto y rango que acepta el driver.
DEFAULT_PERIOD_S = 0.020
PERIOD_RANGE_S = (0.005, 0.050)


def generate_launch_description():
    share = FindPackageShare('movemaster_control')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false', choices=['true', 'false'],
            description='true: mock_components/GenericSystem en lugar de los SPARK MAX, '
                        'sin CAN ni motores.'),
        DeclareLaunchArgument(
            'joint_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('movemaster_hardware'), 'config', 'joints.json']),
            description='joints.json: articulaciones, calibración y period_s.'),
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='Interfaz SocketCAN de los SPARK MAX.'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution([share, 'config', 'movemaster_controllers.yaml']),
            description='Parámetros del controller_manager y de los controladores.'),
        DeclareLaunchArgument(
            'description_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('movemaster_description'), 'urdf', 'movemaster.urdf.xacro']),
            description='Xacro del robot; recibe joint_config, can_interface y '
                        'use_mock_hardware.'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    joint_config = Path(arg('joint_config'))
    config = _read_joint_config(joint_config)
    joints = list(config['joints'])
    update_rate = round(1.0 / config.get('period_s', DEFAULT_PERIOD_S))
    use_mock_hardware = arg('use_mock_hardware') == 'true'

    robot_description = xacro.process_file(arg('description_file'), mappings={
        'joint_config': str(joint_config),
        'can_interface': arg('can_interface'),
        'use_mock_hardware': arg('use_mock_hardware'),
    }).toxml()
    derived_parameters = _write_parameters({
        'controller_manager': {'ros__parameters': {'update_rate': update_rate}},
        'joint_trajectory_controller': {'ros__parameters': {'joints': joints}},
    })

    hardware = ('hardware simulado' if use_mock_hardware
                else f'MovemasterHardware en {arg("can_interface")}')
    return [
        LogInfo(msg=f'MoveMaster: {", ".join(joints)} a {update_rate} Hz con {hardware}.'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='both', parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager', executable='ros2_control_node', output='both',
            # Los valores derivados de joints.json van después y sobrescriben al YAML.
            parameters=[arg('controllers_file'), derived_parameters],
            # Jazzy 4.0 lee ~/robot_description; las versiones recientes, robot_description.
            remappings=[('~/robot_description', '/robot_description')],
        ),
        Node(
            package='controller_manager', executable='spawner', output='both',
            arguments=[
                'joint_state_broadcaster', 'joint_trajectory_controller',
                '--controller-manager', '/controller_manager',
                # Los servicios del controller_manager aparecen con el hardware ya activo;
                # si el CAN no responde, el spawner falla en vez de esperar sin fin.
                '--controller-manager-timeout', '60',
            ],
        ),
    ]


def _read_joint_config(path):
    try:
        config = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        raise RuntimeError(f'No se puede leer la configuración de ejes {path}: {error}') from error
    if not isinstance(config.get('joints'), dict) or not config['joints']:
        raise RuntimeError(f'{path}: "joints" debe ser un objeto con al menos una articulación.')
    period_s = config.get('period_s', DEFAULT_PERIOD_S)
    if not isinstance(period_s, (int, float)) or not (
            PERIOD_RANGE_S[0] <= period_s <= PERIOD_RANGE_S[1]):
        raise RuntimeError(
            f'{path}: period_s = {period_s} s; el driver acepta de {PERIOD_RANGE_S[0]} '
            f'a {PERIOD_RANGE_S[1]} s.')
    return config


def _write_parameters(parameters):
    with tempfile.NamedTemporaryFile(
            'w', prefix='movemaster_control_', suffix='.yaml', delete=False) as file:
        yaml.safe_dump(parameters, file)
    return file.name
