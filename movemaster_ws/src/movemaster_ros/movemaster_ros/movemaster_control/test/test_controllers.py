"""The controllers use the interfaces MovemasterHardware exports.

The robot description, with its ros2_control block, is tested in movemaster_description.
"""

from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]


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
