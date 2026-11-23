import json
import math
from pathlib import Path

import pytest

from movemaster_hmi.robot_model import (check_speed, check_target, Joint, load_jog_axes,
                                        load_joints, move_duration)

PACKAGES = Path(__file__).resolve().parents[2]
JOINTS = [Joint('joint_1', -90.0, 90.0, 30.0, math.inf),
          Joint('joint_2', -45.0, 120.0, 10.0, 20.0)]


def test_loads_limits_and_speeds_in_degrees(tmp_path):
    config = tmp_path / 'joints.json'
    config.write_text(json.dumps({'joints': {
        # A SPARK joint in MAXMotion: its profile is slower than max_velocity_rad_s.
        'joint_1': {'gear_ratio': 100.0, 'min_position_rad': -math.pi / 2,
                    'max_position_rad': math.pi, 'max_velocity_rad_s': 21.0,
                    'control': {'mode': 'maxmotion', 'slot': 0},
                    'slots': {'0': {'maxmotion': {'cruise_velocity': 1000,
                                                  'max_acceleration': 500}}}},
        # The same joint in Position: only max_velocity_rad_s limits it.
        'joint_2': {'gear_ratio': 100.0, 'min_position_rad': -1.0, 'max_position_rad': 1.0,
                    'max_velocity_rad_s': 0.5, 'control': {'mode': 'position', 'slot': 0},
                    'slots': {'0': {'maxmotion': {'cruise_velocity': 1000,
                                                  'max_acceleration': 500}}}},
        # A joint that is not a SPARK.
        'joint_5': {'device': 'gripper', 'min_position_rad': -6.28, 'max_position_rad': 6.28,
                    'max_velocity_rad_s': 3.14, 'max_acceleration_rad_s2': 1.57},
    }}))
    first, second, fifth = load_joints(config)
    assert (first.name, first.lower_deg, first.upper_deg) == ('joint_1', -90.0, 180.0)
    assert first.max_speed_deg_s == pytest.approx(60.0)        # 1000 RPM / 100
    assert first.max_acceleration_deg_s2 == pytest.approx(30.0)
    assert second.max_speed_deg_s == pytest.approx(math.degrees(0.5))
    assert second.max_acceleration_deg_s2 == math.inf
    assert fifth.max_acceleration_deg_s2 == pytest.approx(math.degrees(1.57))


def test_the_repository_configuration_loads():
    joints = load_joints(PACKAGES / 'movemaster_hardware' / 'config' / 'joints.json')
    assert [joint.name for joint in joints][:2] == ['joint_1', 'joint_2']
    assert all(joint.lower_deg < 0 < joint.upper_deg and joint.max_speed_deg_s > 0
               for joint in joints)
    x_joints, y_joints = load_jog_axes(PACKAGES / 'movemaster_teleop' / 'config' / 'jog.yaml')
    assert set(x_joints) | set(y_joints) == {1, 2, 3, 4, 5}
    assert load_jog_axes(PACKAGES / 'missing.yaml') == ((4, 5), (1, 2, 3))


def test_a_target_must_be_within_the_limits():
    check_target(JOINTS, [-90.0, 120.0])
    with pytest.raises(ValueError, match='J2 must be between -45.0 and 120.0'):
        check_target(JOINTS, [0.0, 120.1])
    with pytest.raises(ValueError, match='J1'):
        check_target(JOINTS, [math.nan, 0.0])
    with pytest.raises(ValueError, match='Expected 2'):
        check_target(JOINTS, [0.0])


def test_speed_is_a_percentage_above_zero():
    check_speed(100)
    for bad in (0, -5, 100.1, math.nan):
        with pytest.raises(ValueError):
            check_speed(bad)


def test_the_slowest_joint_sets_the_duration():
    # joint_1: 60 degrees at 30 degrees/s, peak 1.5 times the mean: 3 s.
    assert move_duration(JOINTS, [0, 0], [60, 0], 100) == pytest.approx(3.0)
    assert move_duration(JOINTS, [0, 0], [60, 0], 50) == pytest.approx(6.0)
    # joint_2: 20 degrees at 10 degrees/s need 3 s too, and its acceleration allows it:
    # sqrt(6 * 20 / 20) = 2.45 s.
    assert move_duration(JOINTS, [0, 0], [0, -20], 100) == pytest.approx(3.0)
    # A short move is limited by acceleration: sqrt(6 * 0.3 / 20) = 0.3 s > 1.5 * 0.3 / 10.
    assert move_duration(JOINTS, [0, 0], [0, 0.3], 100) == pytest.approx(0.3)
    # No move at all still gets the minimum time.
    assert move_duration(JOINTS, [5, 5], [5, 5], 100) == pytest.approx(0.2)
