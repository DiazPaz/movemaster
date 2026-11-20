"""The bringup launch, its homing step, what it includes and the systemd service."""

import importlib.util
import math
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
PACKAGES = PACKAGE.parent
JOINT_CONFIG = PACKAGES / 'movemaster_hardware' / 'config' / 'joints.json'
DH_CONFIG = PACKAGES / 'movemaster_description' / 'config' / 'dh.yaml'


def load_module():
    path = PACKAGE / 'launch' / 'bringup.launch.py'
    spec = importlib.util.spec_from_file_location('movemaster_bringup_launch', path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    return launch


def setup(**arguments):
    """The actions the launch starts with, for these arguments over the defaults."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    launch = load_module()
    context = LaunchContext()
    for entity in launch.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = ''.join(
                s.perform(context) for s in entity.default_value)
    context.launch_configurations['joint_config'] = str(JOINT_CONFIG)
    context.launch_configurations.update(arguments)
    return launch._launch_setup(context)


def control_arguments(actions):
    """launch_arguments of the movemaster_control include among actions: the one with
    require_reference."""
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    context = LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))
    included = [{text(k): text(v) for k, v in a.launch_arguments}
                for a in actions if isinstance(a, IncludeLaunchDescription)]
    control = [arguments for arguments in included if 'require_reference' in arguments]
    assert len(control) == 1
    return control[0]


def test_arguments_and_defaults():
    from launch.actions import DeclareLaunchArgument
    arguments = {entity.name: entity for entity in load_module().generate_launch_description().entities
                 if isinstance(entity, DeclareLaunchArgument)}
    assert set(arguments) == {'pendant', 'rviz', 'homing', 'use_mock_hardware', 'joint_config',
                              'can_interface'}

    def default(name):
        return ''.join(s.text for s in arguments[name].default_value)
    # By default: the real arm, homed, without pendant or RViz, as the service on the Pi runs it.
    assert default('pendant') == ''
    assert default('rviz') == 'false'
    assert default('homing') == 'true'
    assert default('use_mock_hardware') == 'false'
    assert default('can_interface') == 'can0'


def test_home_pose_comes_from_dh_yaml():
    # Home: arm upright, forearm level (theta_2 = -90 deg, theta_3 = +90 deg).
    pose = dict(item.split('=') for item in load_module().home_pose(JOINT_CONFIG, DH_CONFIG).split(','))
    assert list(pose) == ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
    assert float(pose['joint_2']) == pytest.approx(-math.pi / 2, abs=1e-6)
    assert float(pose['joint_3']) == pytest.approx(math.pi / 2, abs=1e-6)
    assert all(float(pose[name]) == 0 for name in ('joint_1', 'joint_4', 'joint_5'))


def test_home_pose_needs_every_joint_in_dh_yaml(tmp_path):
    joints = tmp_path / 'joints.json'
    joints.write_text('{"joints": {"joint_1": {}, "joint_6": {}}}')
    with pytest.raises(RuntimeError, match='joint_6'):
        load_module().home_pose(joints, DH_CONFIG)


def test_homing_runs_before_the_controller_manager():
    from launch import LaunchContext
    from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
    from launch.utilities import perform_substitutions
    actions = setup(pendant='/dev/ttyUSB0')
    # Only spark_home starts; the rest waits for it.
    assert not any(isinstance(a, IncludeLaunchDescription) for a in actions)
    process = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(process) == 1
    context = LaunchContext()
    assert perform_substitutions(context, process[0].name) == 'spark_home'
    assert len([a for a in actions if isinstance(a, RegisterEventHandler)]) == 1
    # The executable (cmd[0]) is found in the installed package; the arguments are plain text.
    arguments = [perform_substitutions(context, part) for part in process[0].cmd[1:]]
    assert arguments[0].endswith('spec/spark-frames-2.1.0')
    assert arguments[1:3] == [str(JOINT_CONFIG), 'can0']
    assert arguments[3:5] == ['--home', '--pose']
    assert arguments[5] == load_module().home_pose(JOINT_CONFIG, DH_CONFIG)


def test_after_homing_starts_the_robot_only_if_it_succeeded():
    from launch.actions import Shutdown
    launch = load_module()
    robot = ['control', 'jog']
    assert launch.after_homing(0, robot)[-2:] == robot
    failed = launch.after_homing(1, robot)
    assert not set(robot) & set(failed) and any(isinstance(a, Shutdown) for a in failed)


def test_the_plugin_requires_the_reference_only_after_homing():
    from launch.actions import ExecuteProcess, IncludeLaunchDescription
    from launch_ros.actions import Node
    # Without homing (or with the mock), the robot starts at once and takes the power-up pose.
    for arguments in ({'homing': 'false'}, {'use_mock_hardware': 'true'}):
        actions = setup(**arguments)
        assert not any(isinstance(a, ExecuteProcess) for a in actions)
        assert control_arguments(actions)['require_reference'] == 'false'
    # After homing, a joint without reference is a fault.
    arguments = {'pendant': '/dev/ttyUSB0', 'rviz': 'true', 'use_mock_hardware': 'false',
                 'joint_config': str(JOINT_CONFIG), 'can_interface': 'can0'}
    robot = load_module().robot_actions(arguments, require_reference=True)
    assert control_arguments(robot)['require_reference'] == 'true'
    assert len([a for a in robot if isinstance(a, IncludeLaunchDescription)]) == 2  # with jog
    assert any(isinstance(a, Node) for a in robot)  # RViz


def test_included_files_exist():
    # The launch finds them in the installed packages; here, in the source tree.
    for path in ('movemaster_control/launch/movemaster_control.launch.py',
                 'movemaster_teleop/launch/jog.launch.py',
                 'movemaster_description/rviz/display.rviz',
                 'movemaster_description/config/dh.yaml',
                 'movemaster_hardware/config/joints.json',
                 'movemaster_hardware/spec/spark-frames-2.1.0'):
        assert (PACKAGES / path).is_file(), path


def test_service():
    settings = {}  # key -> every value: systemd allows a key several times
    for line in (PACKAGE / 'systemd' / 'movemaster.service').read_text().splitlines():
        if '=' in line and not line.startswith('#'):
            key, value = line.split('=', 1)
            settings.setdefault(key, []).append(value)
    assert any('ros2 launch movemaster_bringup bringup.launch.py' in v
               for v in settings['ExecStart'])
    assert any('type can bitrate 1000000' in v for v in settings['ExecStartPre'])
    assert settings['KillSignal'] == ['SIGINT']    # stops like Ctrl+C: the heartbeat ends
    assert settings['LimitRTPRIO'] == ['99']       # limits.conf does not apply to services
    # A restart after an emergency stop would home the arm again: it would move by itself.
    assert settings['Restart'] == ['no']
