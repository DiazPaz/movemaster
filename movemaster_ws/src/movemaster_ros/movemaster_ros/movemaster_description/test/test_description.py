"""The URDF follows the DH table and joints.json, and carries what the plugin requires."""

import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
DH_CONFIG = PACKAGE / 'config' / 'dh.yaml'
CHAIN = yaml.safe_load(DH_CONFIG.read_text())['chain']
HOME = {row['joint']: math.radians(row['home_deg']) for row in CHAIN}
JOINT_CONFIG = Path(get_package_share_directory('movemaster_hardware'), 'config', 'joints.json')
CONFIG = json.loads(JOINT_CONFIG.read_text())

# Three axes with a range that excludes 0 and numbers PyYAML reads as text.
# Only the fields the description uses; the plugin validates the rest.
THREE_AXES = """{"joints": {
  "joint_1": {"min_position_rad": -3.1416, "max_position_rad": 3.1416, "max_velocity_rad_s": 21},
  "joint_2": {"min_position_rad": -1.2, "max_position_rad": 9e-1, "max_velocity_rad_s": 1.5},
  "joint_3": {"min_position_rad": 0.2, "max_position_rad": 2.5, "max_velocity_rad_s": 2e0}}}"""


def robot(use_mock_hardware=False, joint_config=JOINT_CONFIG, preview=False):
    urdf = xacro.process_file(str(PACKAGE / 'urdf' / 'movemaster.urdf.xacro'), mappings={
        'joint_config': str(joint_config),
        'dh_config': str(DH_CONFIG),
        'use_mock_hardware': 'true' if use_mock_hardware else 'false',
        'preview': 'true' if preview else 'false',
    }).toxml()
    return ET.fromstring(urdf)


@pytest.fixture
def three_axes(tmp_path):
    path = tmp_path / 'joints.json'
    path.write_text(THREE_AXES)
    return path, json.loads(THREE_AXES)['joints']


# 4x4 homogeneous transforms as nested lists.
def multiply(*matrices):
    result = [[float(i == j) for j in range(4)] for i in range(4)]
    for m in matrices:
        result = [[sum(result[i][k] * m[k][j] for k in range(4)) for j in range(4)]
                  for i in range(4)]
    return result


def translation(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def dh_tool0(q):
    """base_link → tool0 straight from the table: Rz(θ) · Tz(d) · Tx(a) · Rx(α) per row."""
    return multiply(*(multiply(
        rot_z(q[row['joint']]), translation(row['a_mm'] / 1000, 0, row['d_mm'] / 1000),
        rot_x(math.radians(row['alpha_deg']))) for row in CHAIN))


def urdf_tool0(root, q):
    """base_link → tool0 by walking the URDF joints; revolute joints take q[name]."""
    by_child = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
    chain, link = [], 'tool0'
    while link != 'base_link':
        chain.insert(0, by_child[link])
        link = chain[0].find('parent').get('link')
    transforms = []
    for joint in chain:
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz').split()] if origin is not None else [0] * 3
        roll, pitch, yaw = (
            [float(v) for v in origin.get('rpy').split()] if origin is not None else [0] * 3)
        transforms += [translation(*xyz), rot_z(yaw), rot_y(pitch), rot_x(roll)]
        if joint.get('type') == 'revolute':
            assert joint.find('axis').get('xyz') == '0 0 1'
            transforms.append(rot_z(q[joint.get('name')]))
    return multiply(*transforms)


def assert_same_pose(actual, expected):
    for row_a, row_e in zip(actual, expected):
        assert row_a == pytest.approx(row_e, abs=1e-9)


def test_home_puts_tool0_where_the_table_says():
    pose = urdf_tool0(robot(preview=True), HOME)
    # Home (0, -90, 90, 0, -90): forearm level at 470 mm, wrist 215 mm down, tool facing down.
    assert [pose[i][3] for i in range(3)] == pytest.approx([0.160, 0, 0.255], abs=1e-9)
    assert [pose[i][2] for i in range(3)] == pytest.approx([0, 0, -1], abs=1e-9)


def test_urdf_chain_matches_the_dh_product():
    root = robot(preview=True)
    rng = random.Random(7)
    poses = [HOME, {name: 0.0 for name in HOME}] + [
        {name: rng.uniform(-math.pi, math.pi) for name in HOME} for _ in range(20)]
    for q in poses:
        assert_same_pose(urdf_tool0(root, q), dh_tool0(q))


def test_preview_moves_every_axis_without_ros2_control():
    root = robot(preview=True)
    revolute = [j.get('name') for j in root.findall('joint') if j.get('type') == 'revolute']
    assert revolute == [row['joint'] for row in CHAIN]
    assert root.find('ros2_control') is None


def test_axes_of_joints_json_move_and_the_rest_stay_at_home(three_axes):
    path, joints = three_axes
    root = robot(joint_config=path)
    for row in CHAIN:
        urdf_joint = root.find(f"joint[@name='{row['joint']}']")
        if row['joint'] in joints:
            config = joints[row['joint']]
            assert urdf_joint.get('type') == 'revolute'
            limit = urdf_joint.find('limit')
            assert float(limit.get('lower')) == pytest.approx(config['min_position_rad'])
            assert float(limit.get('upper')) == pytest.approx(config['max_position_rad'])
            assert float(limit.get('velocity')) == pytest.approx(config['max_velocity_rad_s'])
        else:
            assert urdf_joint.get('type') == 'fixed'
    # With the moving axes at Home, tool0 lands where the full chain puts it.
    assert_same_pose(urdf_tool0(root, HOME), dh_tool0(HOME))


def test_axes_missing_from_dh_yaml_are_rejected(tmp_path):
    path = tmp_path / 'joints.json'
    path.write_text('{"joints": {"joint_1": {}, "joint_6": {}}}')
    with pytest.raises(xacro.XacroException, match='joint_6'):
        robot(joint_config=path)


@pytest.mark.parametrize('use_mock_hardware', [False, True])
def test_ros2_control_declares_the_joints_of_joints_json(use_mock_hardware):
    root = robot(use_mock_hardware)
    control_joints = root.findall('ros2_control/joint')
    # MovemasterHardware::on_init requires exactly these joints.
    assert [joint.get('name') for joint in control_joints] == list(CONFIG['joints'])
    urdf_joints = {j.get('name'): j.get('type') for j in root.findall('joint')}
    for joint in control_joints:
        # ros2_control's component parser rejects joints missing from the URDF.
        assert urdf_joints[joint.get('name')] == 'revolute'
        assert [i.get('name') for i in joint.findall('command_interface')] == ['position']
        assert [i.get('name') for i in joint.findall('state_interface')] == [
            'position', 'velocity', 'current']


def test_real_hardware_receives_the_plugin_parameters():
    hardware = robot().find('ros2_control/hardware')
    assert hardware.findtext('plugin') == 'movemaster_hardware/MovemasterHardware'
    parameters = {p.get('name'): p.text for p in hardware.findall('param')}
    assert Path(parameters['joint_config_path']) == JOINT_CONFIG
    assert Path(parameters['spec_path']).is_file()
    assert parameters['can_interface'] == 'can0'


def test_mock_hardware_starts_at_home_inside_the_limits(three_axes):
    path, joints = three_axes
    root = robot(use_mock_hardware=True, joint_config=path)
    assert root.findtext('ros2_control/hardware/plugin') == 'mock_components/GenericSystem'
    assert not root.findall('ros2_control/hardware/param')
    for name, joint in joints.items():
        initial = root.find(
            f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param")
        assert initial.get('name') == 'initial_value'
        # joint_2 has Home (-90°) below its range, so it starts at the lower limit.
        expected = min(max(HOME[name], joint['min_position_rad']), joint['max_position_rad'])
        assert float(initial.text) == pytest.approx(expected)
