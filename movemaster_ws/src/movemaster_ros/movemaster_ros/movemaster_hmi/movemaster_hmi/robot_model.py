"""What the HMI knows about the arm, read from joints.json; without ROS or Qt so it can be
tested alone. The HMI works in degrees with Home at 0; ROS, in radians with the same zero.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path

import yaml

# A move follows a cubic from rest to rest: its peak speed is 1.5 times the mean and its peak
# acceleration 6 * distance / time^2.
_PEAK_SPEED = 1.5
_PEAK_ACCELERATION = 6.0
MIN_MOVE_S = 0.2


@dataclass(frozen=True)
class Joint:
    name: str
    lower_deg: float
    upper_deg: float
    max_speed_deg_s: float    # the speed of a move at 100 %
    max_acceleration_deg_s2: float  # inf: not limited


def _limits_of_motion(joint):
    """Speed and acceleration the joint is configured to reach, in rad and s."""
    speed = joint['max_velocity_rad_s']
    acceleration = joint.get('max_acceleration_rad_s2', math.inf)
    control = joint.get('control', {})
    profile = joint.get('slots', {}).get(str(control.get('slot')), {}).get('maxmotion')
    if control.get('mode') == 'maxmotion' and profile:
        # The SPARK profiles in motor RPM and RPM/s: the joint cannot go faster than that.
        to_joint = 2 * math.pi / 60 / joint['gear_ratio']
        speed = min(speed, profile['cruise_velocity'] * to_joint)
        acceleration = min(acceleration, profile['max_acceleration'] * to_joint)
    return speed, acceleration


def load_joints(path):
    """The joints of joints.json, in its order: the order of every list of angles."""
    joints = []
    for name, joint in json.loads(Path(path).read_text())['joints'].items():
        speed, acceleration = _limits_of_motion(joint)
        joints.append(Joint(name, math.degrees(joint['min_position_rad']),
                            math.degrees(joint['max_position_rad']), math.degrees(speed),
                            math.degrees(acceleration)))
    return joints


def load_jog_axes(path, default=((4, 5), (1, 2, 3))):
    """Joint numbers the pendant's joystick X and Y jog (jog.yaml), to show its selection."""
    try:
        parameters = yaml.safe_load(Path(path).read_text())['jog_node']['ros__parameters']
        return tuple(parameters['x_joints']), tuple(parameters['y_joints'])
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        return default


def check_target(joints, angles_deg):
    """Raises ValueError unless there is one finite angle per joint, within its limits."""
    if len(angles_deg) != len(joints):
        raise ValueError(f'Expected {len(joints)} joint angles, got {len(angles_deg)}.')
    for number, (joint, angle) in enumerate(zip(joints, angles_deg), 1):
        if not math.isfinite(angle) or not joint.lower_deg <= angle <= joint.upper_deg:
            raise ValueError(f'J{number} must be between {joint.lower_deg:.1f} and '
                             f'{joint.upper_deg:.1f} degrees.')


def check_speed(percent):
    if not math.isfinite(percent) or not 0 < percent <= 100:
        raise ValueError('Speed must be above 0 and at most 100 %.')


def move_duration(joints, start_deg, goal_deg, speed_percent):
    """Seconds for a rest-to-rest move in which no joint passes its speed or acceleration.

    Every joint takes the same time, so they start and stop together.
    """
    duration = MIN_MOVE_S
    for joint, start, goal in zip(joints, start_deg, goal_deg):
        distance = abs(goal - start)
        speed = joint.max_speed_deg_s * speed_percent / 100
        duration = max(duration, _PEAK_SPEED * distance / speed,
                       math.sqrt(_PEAK_ACCELERATION * distance / joint.max_acceleration_deg_s2))
    return duration
