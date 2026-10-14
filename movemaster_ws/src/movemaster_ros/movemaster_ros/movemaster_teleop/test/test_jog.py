import math

import pytest

from movemaster_teleop.jog import Jogger, JointLimits, shape
from movemaster_teleop.pendant_frame import PendantFrame

DT = 0.02
LIMITS = {
    'joint_1': JointLimits(-1.0, 1.0, 0.2),
    'joint_2': JointLimits(-1.0, 1.0, 0.1),
}


def jogger():
    return Jogger(['joint_1', 'joint_2'], LIMITS, x_joints=[1, 5], y_joints=[2, 3, 4],
                  deadband=0.1, max_lead=0.05, idle_timeout=0.5)


def test_shape():
    assert shape(0.05, 0.1) == 0.0
    assert shape(1.0, 0.1) == 1.0
    assert shape(-1.0, 0.1) == -1.0
    assert shape(0.55, 0.1) == pytest.approx(0.25)  # half the travel: a quarter of the speed


def test_idle_joystick_publishes_nothing():
    measured = {'joint_1': 0.0, 'joint_2': 0.0}
    assert jogger().update(PendantFrame(x_joint=1, y_joint=2), measured, DT) is None


def test_starts_from_measured_and_moves_proportionally():
    jog = jogger()
    measured = {'joint_1': 0.3, 'joint_2': -0.2}
    targets = jog.update(PendantFrame(x_joint=1, y_joint=2, x=1000, y=-1000), measured, DT)
    assert targets == pytest.approx([0.3 + 0.2 * DT, -0.2 - 0.1 * DT])


def test_axis_mapping():
    jog = jogger()
    measured = {'joint_1': 0.0, 'joint_2': 0.0}
    # joint_2 is not an X joint and joint_1 is not a Y joint: nothing moves.
    assert jog.update(PendantFrame(x_joint=2, y_joint=1, x=1000, y=1000), measured, DT) is None
    # Selected joints without a SPARK (not in joints.json) are ignored.
    assert jog.update(PendantFrame(x_joint=5, y_joint=3, x=1000, y=1000), measured, DT) is None


def test_target_never_leads_the_joint_more_than_max_lead():
    jog = jogger()
    measured = {'joint_1': 0.0, 'joint_2': 0.0}  # the joint does not move: blocked
    frame = PendantFrame(x_joint=1, x=1000)
    for _ in range(100):
        targets = jog.update(frame, measured, DT)
    assert targets[0] == pytest.approx(0.05)
    assert targets[1] == 0.0  # the other joint keeps its hold target


def test_target_stops_at_the_limit():
    jog = jogger()
    frame = PendantFrame(x_joint=1, x=1000)
    measured = {'joint_1': 0.99, 'joint_2': 0.0}
    for _ in range(10):
        targets = jog.update(frame, measured, DT)
        measured['joint_1'] = targets[0]
    assert targets[0] == 1.0


def test_lost_link_stops_and_session_restarts_from_measured():
    jog = jogger()
    measured = {'joint_1': 0.0, 'joint_2': 0.0}
    jog.update(PendantFrame(x_joint=1, x=1000), measured, DT)
    assert jog.update(None, measured, DT) is None  # no frame: no motion
    for _ in range(30):  # longer than idle_timeout
        jog.update(None, measured, DT)
    measured['joint_1'] = 0.7  # for example, the trajectory controller moved the arm
    targets = jog.update(PendantFrame(x_joint=1, x=1000), measured, DT)
    assert targets[0] == pytest.approx(0.7 + 0.2 * DT)


def test_waits_for_joint_states():
    jog = jogger()
    frame = PendantFrame(x_joint=1, x=1000)
    assert jog.update(frame, {}, DT) is None
    assert jog.update(frame, {'joint_1': 0.0, 'joint_2': math.nan}, DT) is None
