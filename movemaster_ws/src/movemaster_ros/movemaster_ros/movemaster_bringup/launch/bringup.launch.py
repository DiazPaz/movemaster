"""MoveMaster con un solo comando.

    ros2 launch movemaster_bringup bringup.launch.py [pendant:=/dev/ttyUSB0] [rviz:=true]
        [homing:=false] [use_mock_hardware:=true] [joint_config:=...] [can_interface:=can0]

Primero el homing (homing:=true, el valor por defecto, y solo con el brazo real):
  spark_home --home   busca el tope de cada eje que perdió su referencia (homing_order de
                      joints.json), le escribe a su SPARK la posición que le corresponde y lleva
                      el brazo a Home (home_deg de dh.yaml). Si todos los ejes conservan su
                      referencia, no mueve nada. El brazo se mueve solo: ten el paro a la mano.
Si el homing termina bien, arrancan:
  movemaster_control  controller_manager con MovemasterHardware, joint_state_broadcaster,
                      joint_trajectory_controller activo y jog_controller inactivo. Con homing,
                      un eje sin referencia es una falla (require_reference:=true).
  movemaster_teleop   jog_node, solo con pendant:=<puerto>. El switch MANUAL del pendant
                      elige el controlador: MANUAL, jog; AUTO, trayectorias.
  rviz2               solo con rviz:=true; necesita pantalla.
Si falla, no arranca nada más y el launch se cierra.

El servicio systemd/movemaster.service lo arranca al encender la Raspberry Pi.
"""

import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
import yaml

# Catálogo de tramas que usa el plugin (spec_path en movemaster.urdf.xacro).
SPARK_FRAMES = 'spark-frames-2.1.0'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pendant', default_value='',
            description='Puerto serie del teach pendant (ESP32), por ejemplo /dev/ttyUSB0 o '
                        '/dev/serial/by-id/...; vacío: sin jog.'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Abrir RViz con el modelo del brazo; necesita pantalla.'),
        DeclareLaunchArgument(
            'homing', default_value='true', choices=['true', 'false'],
            description='true: antes del controller_manager, spark_home --home referencia los '
                        'ejes que la perdieron y lleva el brazo a Home. false: cada eje toma '
                        'como referencia la postura en que se encendió.'),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false', choices=['true', 'false'],
            description='true: hardware simulado, sin CAN ni motores ni homing.'),
        DeclareLaunchArgument(
            'joint_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('movemaster_hardware'), 'config', 'joints.json']),
            description='joints.json: articulaciones, calibración, homing y period_s.'),
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='Interfaz SocketCAN de los SPARK MAX.'),
        OpaqueFunction(function=_launch_setup),
    ])


def home_pose(joint_config, dh_config):
    """--pose de spark_home: el home_deg de dh.yaml de cada eje de joints.json, en rad."""
    joints = json.loads(Path(joint_config).read_text())['joints']
    chain = yaml.safe_load(Path(dh_config).read_text())['chain']
    home = {row['joint']: row['home_deg'] for row in chain}
    missing = [name for name in joints if name not in home]
    if missing:
        raise RuntimeError(f'{", ".join(missing)}: no están en {dh_config}, sin Home.')
    return ','.join(f'{name}={math.radians(home[name]):.6f}' for name in joints)


def after_homing(returncode, robot):
    """Lo que arranca cuando termina spark_home: el robot, o nada si el homing falló."""
    if returncode == 0:
        return [LogInfo(msg='Homing listo: arranca el controller_manager.')] + robot
    return [
        LogInfo(msg=f'spark_home --home terminó con código {returncode}: no se arranca el '
                    'controller_manager. Su salida, arriba, dice por qué.'),
        Shutdown(reason='falló el homing'),
    ]


def robot_actions(arguments, require_reference):
    """controller_manager, jog del pendant y RViz, según los argumentos del launch."""
    joint_config = arguments['joint_config']
    robot = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('movemaster_control'), 'launch',
                 'movemaster_control.launch.py'])),
            launch_arguments={
                'use_mock_hardware': arguments['use_mock_hardware'],
                'joint_config': joint_config,
                'can_interface': arguments['can_interface'],
                'require_reference': 'true' if require_reference else 'false',
            }.items()),
    ]
    if arguments['pendant']:
        robot.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('movemaster_teleop'), 'launch', 'jog.launch.py'])),
            launch_arguments={'port': arguments['pendant'], 'joint_config': joint_config}.items()))
    if arguments['rviz'] == 'true':
        robot.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('movemaster_description'), 'rviz', 'display.rviz'])]))
    return robot


def _launch_setup(context):
    arguments = {name: LaunchConfiguration(name).perform(context) for name in (
        'pendant', 'rviz', 'homing', 'use_mock_hardware', 'joint_config', 'can_interface')}
    homing = arguments['homing'] == 'true' and arguments['use_mock_hardware'] == 'false'
    robot = robot_actions(arguments, require_reference=homing)
    if not homing:
        return robot

    joint_config = arguments['joint_config']
    hardware = Path(get_package_share_directory('movemaster_hardware'))
    dh_config = Path(get_package_share_directory('movemaster_description'), 'config', 'dh.yaml')
    spark_home = ExecuteProcess(
        name='spark_home', output='screen',
        cmd=[ExecutableInPackage('spark_home', 'movemaster_hardware'),
             str(hardware / 'spec' / SPARK_FRAMES), joint_config, arguments['can_interface'],
             '--home', '--pose', home_pose(joint_config, dh_config)])
    return [
        LogInfo(msg='Homing: el brazo se mueve solo hasta Home; ten el paro de emergencia a '
                    'la mano.'),
        spark_home,
        RegisterEventHandler(OnProcessExit(
            target_action=spark_home,
            on_exit=lambda event, _context: after_homing(event.returncode, robot))),
    ]
