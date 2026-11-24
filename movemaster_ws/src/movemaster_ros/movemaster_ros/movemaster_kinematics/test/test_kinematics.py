import math
from pathlib import Path
import random

import pytest
import yaml

from movemaster_kinematics import cli
from movemaster_kinematics.kinematics import Arm, JointLimits, Pose, Unreachable

ARM = Arm.from_files()
DH = yaml.safe_load(Path(__file__).resolve().parents[2].joinpath(
    'movemaster_description', 'config', 'dh.yaml').read_text())
FREE = {row['joint']: JointLimits(-math.pi, math.pi, 1.0) for row in DH['chain']}


def position(pose):
    return [pose.x, pose.y, pose.z]


def random_q(arm, rng):
    return [rng.uniform(limit.lower, limit.upper) for limit in arm.limits]


def test_home_points_the_tool_down_below_the_forearm():
    # README of movemaster_description: tool0 at (157.4, 0, 320) mm, faceplate down.
    home = ARM.fk([0.0] * 5)
    assert position(home) == pytest.approx([0.1574, 0, 0.320 - ARM.tool_length], abs=1e-12)
    assert home.pitch == pytest.approx(-math.pi / 2)
    assert home.roll == pytest.approx(0)


def test_dh_zero_pose_stretches_upper_arm_and_forearm_forward():
    # Every θ = 0, q = (0, 90°, -90°, 0, 0): tool0 at (377.4, 0, 100) mm, still pointing down.
    pose = ARM.fk([0, math.pi / 2, -math.pi / 2, 0, 0])
    assert position(pose) == pytest.approx([0.3774, 0, 0.100 - ARM.tool_length], abs=1e-12)
    assert pose.pitch == pytest.approx(-math.pi / 2)


def test_the_tool_length_comes_from_dh_yaml():
    assert ARM.tool_length == DH['tool']['length_mm'] / 1000
    longer = Arm(DH['chain'], 0.165, FREE)
    assert longer.fk([0] * 5).z == pytest.approx(ARM.fk([0] * 5).z - (0.165 - ARM.tool_length))


def test_pitch_and_roll_follow_the_wrist():
    # Wrist pitch tilts the tool forward; the faceplate turns it about its axis.
    assert ARM.fk([0, 0, 0, -math.pi / 2, 0]).pitch == pytest.approx(0, abs=1e-12)
    assert ARM.fk([0, 0, 0, 0, 0.7]).roll == pytest.approx(0.7)
    # The waist turns the whole pose about z: pitch and roll do not change.
    turned = ARM.fk([1.0, 0.3, -0.4, -0.5, 0.2])
    straight = ARM.fk([0.0, 0.3, -0.4, -0.5, 0.2])
    assert math.hypot(turned.x, turned.y) == pytest.approx(math.hypot(straight.x, straight.y))
    assert (turned.pitch, turned.roll) == pytest.approx((straight.pitch, straight.roll))


def test_ik_returns_the_joints_it_starts_from():
    rng = random.Random(7)
    for _ in range(2000):
        q = random_q(ARM, rng)
        assert ARM.ik(ARM.fk(q), near=q) == pytest.approx(q, abs=1e-9)


def test_ik_from_home_reaches_every_reachable_pose_within_the_limits():
    rng = random.Random(11)
    for _ in range(2000):
        pose = ARM.fk(random_q(ARM, rng))
        q = ARM.ik(pose)
        assert all(limit.lower - 1e-9 <= angle <= limit.upper + 1e-9
                   for angle, limit in zip(q, ARM.limits))
        reached = ARM.fk(q)
        assert position(reached) == pytest.approx(position(pose), abs=1e-9)
        assert (reached.pitch, reached.roll) == pytest.approx((pose.pitch, pose.roll), abs=1e-9)


def test_of_two_solutions_the_closest_wins():
    # Elbow up and elbow down reach the same pose; the joints go to the one nearer to them.
    arm = Arm(DH['chain'], ARM.tool_length, FREE)
    up = [0.2, 0.4, -0.8, 0.3, 0.1]
    pose = arm.fk(up)
    down = arm.ik(pose, near=[0.2, 1.2, -2.3, 0.3, 0.1])
    assert down != pytest.approx(up, abs=0.1)
    assert position(arm.fk(down)) == pytest.approx(position(pose), abs=1e-9)
    assert arm.ik(pose, near=up) == pytest.approx(up, abs=1e-9)
    assert arm.ik(pose, near=down) == pytest.approx(down, abs=1e-9)


def test_the_faceplate_stays_on_the_turn_it_is_on():
    # J5 has two turns of range: a roll is reached on the turn closest to where it is.
    q = ARM.ik(Pose(0.2, 0.05, 0.2, -math.pi / 2, -0.3), near=[0, 0, 0, 0, 6.0])
    assert q[4] == pytest.approx(2 * math.pi - 0.3)


def test_a_point_behind_is_reached_over_the_back():
    # The waist does not turn half a revolution; the shoulder leans back instead.
    pose = ARM.fk([0.0, -0.45, -1.55, -0.5, 0.0])
    assert pose.x < 0  # behind the waist axis, pointing back and up
    q = ARM.ik(pose)
    assert abs(q[0]) < 1e-9 and q[1] < 0
    assert position(ARM.fk(q)) == pytest.approx(position(pose), abs=1e-9)


@pytest.mark.parametrize('pose, reason', [
    (Pose(1.0, 0.0, 0.3, -math.pi / 2, 0.0), 'Out of reach: the wrist would be 657 mm beyond'),
    (Pose(0.0, 0.0, 0.5, 0.0, 0.0), 'on the waist axis'),
])
def test_unreachable_poses_say_why(pose, reason):
    with pytest.raises(Unreachable, match=reason):
        ARM.ik(pose)


def test_a_pose_only_past_a_joint_limit_names_the_joint():
    # The elbow bent past its range: a free arm reaches it, the real one does not.
    pose = Arm(DH['chain'], ARM.tool_length, FREE).fk([0.0, 0.2, 1.2, -0.5, 0.0])
    with pytest.raises(Unreachable, match=r'Out of the joint limits: joint_\d would need'):
        ARM.ik(pose)


def test_only_the_movemaster_shape_is_accepted():
    chain = [dict(row) for row in DH['chain']]
    chain[1]['alpha_deg'] = 90
    with pytest.raises(ValueError, match='MoveMaster shape'):
        Arm(chain, 0.065, FREE)


def test_cli_prints_in_mm_and_degrees(capsys):
    assert cli.main(['fk', '0', '0', '0', '0', '0']) == 0
    assert capsys.readouterr().out.strip() == (
        f'X 157.4 mm  Y 0.0 mm  Z {320 - ARM.tool_length * 1000:.1f} mm  '
        'pitch -90.0 deg  roll 0.0 deg')
    assert cli.main(['ik', '157.4', '0', f'{320 - ARM.tool_length * 1000}', '-90', '0']) == 0
    assert capsys.readouterr().out.strip() == (
        'J1 0.00 deg  J2 0.00 deg  J3 0.00 deg  J4 0.00 deg  J5 0.00 deg')
    assert cli.main(['ik', '1000', '0', '300', '-90', '0']) == 1
    assert 'Out of reach' in capsys.readouterr().err
