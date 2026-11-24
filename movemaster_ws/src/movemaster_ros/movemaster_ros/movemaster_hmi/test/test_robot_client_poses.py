"""The pose half of the HMI's robot API: poses in mm and degrees, joints from the IK."""
import math

import pytest

pytest.importorskip('PySide6')

from movemaster_hmi import robot_client  # noqa: E402
from movemaster_hmi.robot_model import load_joints  # noqa: E402
from movemaster_kinematics.kinematics import default_joint_config  # noqa: E402


class Robot(robot_client.RobotClient):
    """Offline client that keeps what it would send."""

    def __init__(self, *arguments):
        super().__init__(*arguments)
        self.sent = []
        self.mode = robot_client.AUTO

    def _submit(self, *command):
        self.sent.append(command)


@pytest.fixture
def robot():
    config = default_joint_config()
    return Robot(load_joints(config), ((4, 5), (1, 2, 3)), robot_client.load_arm(config))


def test_home_pose_in_mm_and_degrees(robot):
    assert robot.pose_of([0.0] * 5) == pytest.approx([157.4, 0, 320 - 65, -90, 0])
    assert robot.tool_length() == pytest.approx(65)


def test_move_to_pose_sends_the_joints_of_the_pose(robot):
    target = [250.0, 50.0, 200.0, -90.0, 30.0]
    robot.move_to_pose(target, 30)
    kind, angles, speed = robot.sent[-1]
    assert kind == 'move' and speed == 30
    assert robot.pose_of(angles) == pytest.approx(target, abs=1e-6)


def test_an_unreachable_pose_is_refused_with_the_reason(robot):
    with pytest.raises(ValueError, match='Out of reach'):
        robot.move_to_pose([1000.0, 0.0, 300.0, -90.0, 0.0], 30)
    assert robot.sent == [] and not robot.busy


def test_pose_steps_become_waypoints_before_the_program_starts(robot):
    steps = [{'type': 'pose', 'pose': [250.0, 50.0, 200.0, -90.0, 30.0]},
             {'type': 'wait', 'seconds': 1.0},
             {'type': 'pose', 'pose': robot.pose_of([-15.0, 25.0, -40.0, -10.0, 0.0])}]
    robot.run_program(steps, 25)
    kind, sent, speed = robot.sent[-1]
    assert [step['type'] for step in sent] == ['waypoint', 'wait', 'waypoint']
    assert robot.pose_of(sent[2]['joints']) == pytest.approx(steps[2]['pose'], abs=1e-6)
    assert steps[0]['type'] == 'pose'  # the HMI's own program is left as it is


def test_a_program_with_an_unreachable_pose_names_the_step(robot):
    steps = [{'type': 'wait', 'seconds': 1.0},
             {'type': 'pose', 'pose': [0.0, 0.0, 500.0, 0.0, 0.0]}]
    with pytest.raises(ValueError, match='Step 2: The tcp is on the waist axis'):
        robot.run_program(steps, 25)


def test_another_tool_for_the_session(robot):
    robot.set_tool_length(100)
    assert robot.pose_of([0.0] * 5)[2] == pytest.approx(320 - 100)


def test_without_kinematics_poses_are_refused():
    robot = Robot(load_joints(default_joint_config()), ((4, 5), (1, 2, 3)))
    with pytest.raises(robot_client.RobotError, match='movemaster_kinematics'):
        robot.pose_of([0.0] * 5)
    assert math.isfinite(robot.joints[0].lower_deg)
