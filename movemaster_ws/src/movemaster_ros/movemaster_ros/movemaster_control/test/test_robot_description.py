"""The controller_manager model matches joints.json and what the plugin requires."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
JOINT_CONFIG = Path(get_package_share_directory('movemaster_hardware'), 'config', 'joints.json')
CONFIG = json.loads(JOINT_CONFIG.read_text())

# Three axes with a range that excludes 0 and numbers PyYAML reads as text.
# Only the fields the description uses; the plugin validates the rest.
THREE_AXES = """{"joints": {
  "joint_1": {"min_position_rad": -3.1416, "max_position_rad": 3.1416, "max_velocity_rad_s": 21},
  "joint_2": {"min_position_rad": -1.2, "max_position_rad": 9e-1, "max_velocity_rad_s": 1.5},
  "joint_3": {"min_position_rad": 0.2, "max_position_rad": 2.5, "max_velocity_rad_s": 2e0}}}"""


def robot(use_mock_hardware, joint_config=JOINT_CONFIG):
    urdf = xacro.process_file(str(PACKAGE / 'urdf' / 'movemaster.urdf.xacro'), mappings={
        'joint_config': str(joint_config),
        'use_mock_hardware': 'true' if use_mock_hardware else 'false',
    }).toxml()
    return ET.fromstring(urdf)


@pytest.fixture
def three_axes(tmp_path):
    path = tmp_path / 'joints.json'
    path.write_text(THREE_AXES)
    return path, json.loads(THREE_AXES)['joints']


@pytest.mark.parametrize('use_mock_hardware', [False, True])
def test_ros2_control_declares_the_joints_of_joints_json(use_mock_hardware):
    root = robot(use_mock_hardware)
    control_joints = root.findall('ros2_control/joint')
    # MovemasterHardware::on_init requires exactly these joints.
    assert [joint.get('name') for joint in control_joints] == list(CONFIG['joints'])
    urdf_joints = {joint.get('name') for joint in root.findall('joint')}
    for joint in control_joints:
        # ros2_control's component parser rejects joints missing from the URDF.
        assert joint.get('name') in urdf_joints
        assert [i.get('name') for i in joint.findall('command_interface')] == ['position']
        assert [i.get('name') for i in joint.findall('state_interface')] == [
            'position', 'velocity', 'current']


def test_real_hardware_receives_the_plugin_parameters():
    hardware = robot(use_mock_hardware=False).find('ros2_control/hardware')
    assert hardware.findtext('plugin') == 'movemaster_hardware/MovemasterHardware'
    parameters = {p.get('name'): p.text for p in hardware.findall('param')}
    assert Path(parameters['joint_config_path']) == JOINT_CONFIG
    assert Path(parameters['spec_path']).is_file()
    assert parameters['can_interface'] == 'can0'


def test_mock_hardware_starts_inside_the_limits(three_axes):
    path, joints = three_axes
    root = robot(use_mock_hardware=True, joint_config=path)
    assert root.findtext('ros2_control/hardware/plugin') == 'mock_components/GenericSystem'
    assert not root.findall('ros2_control/hardware/param')
    for name, joint in joints.items():
        initial = root.find(
            f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param")
        assert initial.get('name') == 'initial_value'
        assert joint['min_position_rad'] <= float(initial.text) <= joint['max_position_rad']


def test_urdf_chain_and_limits_come_from_joints_json(three_axes):
    path, joints = three_axes
    root = robot(use_mock_hardware=False, joint_config=path)
    parent = 'base_link'
    for name, joint in joints.items():
        urdf_joint = root.find(f"joint[@name='{name}']")
        assert urdf_joint.get('type') == 'revolute'
        assert urdf_joint.find('parent').get('link') == parent
        parent = urdf_joint.find('child').get('link')
        limit = urdf_joint.find('limit')
        assert float(limit.get('lower')) == pytest.approx(joint['min_position_rad'])
        assert float(limit.get('upper')) == pytest.approx(joint['max_position_rad'])
        assert float(limit.get('velocity')) == pytest.approx(joint['max_velocity_rad_s'])
    assert [j.get('name') for j in root.findall('ros2_control/joint')] == list(joints)


def test_controllers_use_the_interfaces_the_plugin_exports():
    parameters = yaml.safe_load((PACKAGE / 'config' / 'movemaster_controllers.yaml').read_text())
    manager = parameters['controller_manager']['ros__parameters']
    assert manager['joint_state_broadcaster']['type'] == (
        'joint_state_broadcaster/JointStateBroadcaster')
    assert manager['joint_trajectory_controller']['type'] == (
        'joint_trajectory_controller/JointTrajectoryController')
    trajectory = parameters['joint_trajectory_controller']['ros__parameters']
    assert trajectory['command_interfaces'] == ['position']
    assert set(trajectory['state_interfaces']) <= {'position', 'velocity'}
    # The launch fills the joints from joints.json.
    assert 'joints' not in trajectory
