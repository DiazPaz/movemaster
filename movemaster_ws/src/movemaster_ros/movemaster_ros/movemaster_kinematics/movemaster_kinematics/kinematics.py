"""Kinematics of the MoveMaster: the pose of the tool tip from the joint positions (fk) and the
joint positions for a pose (ik). Without ROS, so it can be tested and used alone.

The geometry is the one of the URDF: the DH table and the tool of movemaster_description's
dh.yaml, and the joint limits of movemaster_hardware's joints.json. Joint positions are q, as in
ROS (/joint_states, Home at q = 0), in rad; lengths in m.

The arm has five axes, so a pose has five numbers, not six (Pose): the tcp's position, the pitch
of the tool and its roll. The yaw is not free: the tool always lies in the vertical plane through
the waist axis and the tcp, the plane the waist turns.

The inverse kinematics is closed form, like the one of an industrial controller: the waist
points the arm at the tcp (or away from it, reaching over the back), shoulder and elbow place
the wrist as a two-link arm, the wrist pitch gives the tool its pitch and the faceplate its
roll. Of the solutions within the joint limits it returns the one the joints reach first from
where they are.
"""

import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np
import yaml

_EPS = 1e-9
# The MoveMaster's shape, which the closed form relies on: twist of each row of the DH table.
# Only these are fixed; lengths and offsets come from dh.yaml.
_ALPHA_DEG = (-90, 0, 0, -90, 0)


@dataclass(frozen=True)
class Pose:
    """Where the tool tip (tcp) is and how the tool points, in base_link.

    x, y, z   the tcp, in m.
    pitch     elevation of the tool's axis (z of tool0, toward the tip) in the arm's vertical
              plane, in rad: 0 points horizontally away from the waist axis, -pi/2 straight
              down, pi/2 straight up; beyond +-pi/2 it points back toward the waist axis.
    roll      turn of the tool about its axis, in rad, counterclockwise looking at the tip from
              the flange; 0 at Home.
    """

    x: float
    y: float
    z: float
    pitch: float
    roll: float


class Unreachable(ValueError):
    """No joint positions within the limits give this pose; the message says why."""


@dataclass(frozen=True)
class JointLimits:
    lower: float
    upper: float
    max_velocity: float


def _wrap(angle):
    """The same angle in -pi..pi."""
    return math.remainder(angle, 2 * math.pi)


def _dh(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


class Arm:
    """The kinematics of one arm: its DH table, its tool and its joint limits."""

    def __init__(self, chain, tool_length, limits):
        """chain: the rows of dh.yaml; tool_length: from tool0 to the tcp, in m; limits:
        JointLimits by joint name. A joint without limits stays at q = 0, as in the URDF."""
        if [row['alpha_deg'] for row in chain] != list(_ALPHA_DEG) \
                or chain[0]['a_mm'] != 0 or any(row['d_mm'] != 0 for row in chain[1:4]) \
                or chain[3]['a_mm'] != 0 or chain[4]['a_mm'] != 0:
            raise ValueError('The closed-form kinematics needs the MoveMaster shape: waist, '
                             'shoulder, elbow, wrist pitch and roll (alpha -90, 0, 0, -90, 0).')
        self.names = [row['joint'] for row in chain]
        self.offsets = [math.radians(row['theta_offset_deg']) for row in chain]
        self.d = [row['d_mm'] / 1000 for row in chain]
        self.a = [row['a_mm'] / 1000 for row in chain]
        self.alpha = [math.radians(row['alpha_deg']) for row in chain]
        self.tool_length = tool_length
        self.limits = [limits.get(name, JointLimits(0.0, 0.0, 1.0)) for name in self.names]
        self._home_roll = 0.0
        self._home_roll = self._roll_of(self._flange(np.zeros(5)), np.array([1.0, 0.0, 0.0]))

    @classmethod
    def from_files(cls, dh_config=None, joint_config=None):
        """The arm of dh.yaml and joints.json; by default, the installed ones (or, without ROS,
        the ones of this repository)."""
        dh = yaml.safe_load(Path(dh_config or default_dh_config()).read_text())
        joints = json.loads(Path(joint_config or default_joint_config()).read_text())['joints']
        limits = {name: JointLimits(joint['min_position_rad'], joint['max_position_rad'],
                                    joint['max_velocity_rad_s'])
                  for name, joint in joints.items()}
        return cls(dh['chain'], dh['tool']['length_mm'] / 1000, limits)

    def with_tool_length(self, length):
        """The same arm with another tool: tool0 to the tcp, in m."""
        if not (math.isfinite(length) and length >= 0):
            raise ValueError('The tool length must be 0 or more.')
        arm = copy.copy(self)
        arm.tool_length = length
        return arm

    # Forward kinematics

    def _flange(self, q):
        """base_link -> tool0 for joint positions q."""
        transform = np.eye(4)
        for i in range(5):
            transform = transform @ _dh(q[i] + self.offsets[i], self.d[i], self.a[i],
                                        self.alpha[i])
        return transform

    def _roll_of(self, flange, radial):
        """Turn of the tool about its axis, from the horizontal normal to the arm's plane."""
        axis, mark = flange[:3, 2], flange[:3, 0]
        normal = np.cross([0.0, 0.0, 1.0], radial)
        return _wrap(math.atan2(np.dot(np.cross(normal, mark), axis), np.dot(normal, mark))
                     - self._home_roll)

    def fk(self, q):
        """The Pose of the tcp for joint positions q (rad, in the order of names)."""
        flange = self._flange(q)
        axis = flange[:3, 2]
        tcp = flange[:3, 3] + self.tool_length * axis
        radial = self._radial(tcp, q[0] + self.offsets[0])
        pitch = math.atan2(axis[2], np.dot(axis, radial))
        return Pose(*tcp, pitch, self._roll_of(flange, radial))

    @staticmethod
    def _radial(tcp, waist):
        """Horizontal unit vector from the waist axis to the tcp; on the axis, the arm's way."""
        r = math.hypot(tcp[0], tcp[1])
        if r > 1e-6:
            return np.array([tcp[0] / r, tcp[1] / r, 0.0])
        return np.array([math.cos(waist), math.sin(waist), 0.0])

    # Inverse kinematics

    def ik(self, pose, near=None):
        """Joint positions q (rad) that put the tcp at pose, within the joint limits: of all
        the solutions, the one the joints reach first from near (default Home). Raises
        Unreachable when there is none."""
        near = list(near) if near is not None else [0.0] * 5
        p = np.array([pose.x, pose.y, pose.z])
        r = math.hypot(pose.x, pose.y)
        if r < 1e-6:
            raise Unreachable('The tcp is on the waist axis: the waist angle is undefined there.')
        radial = np.array([pose.x / r, pose.y / r, 0.0])
        axis = math.cos(pose.pitch) * radial + np.array([0.0, 0.0, math.sin(pose.pitch)])
        wrist = p - (self.tool_length + self.d[4]) * axis  # center of the wrist pitch axis
        a2, a3 = self.a[1], self.a[2]

        # Shoulder to wrist: what the upper arm and the forearm must span.
        span = math.dist(wrist, (0.0, 0.0, self.d[0]))
        if span > a2 + a3 + _EPS:
            raise Unreachable(f'Out of reach: the wrist would be {(span - a2 - a3) * 1000:.0f} '
                              'mm beyond what the arm reaches.')
        if span < abs(a2 - a3) - _EPS:
            raise Unreachable('Out of reach: the wrist would be too close to the shoulder.')
        cos_elbow = max(-1.0, min(1.0, (span ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)))

        solutions, out_of_range = [], []
        for waist in (math.atan2(pose.y, pose.x), math.atan2(pose.y, pose.x) + math.pi):
            # In the arm's vertical plane: forward along the waist, up along z.
            forward = np.array([math.cos(waist), math.sin(waist), 0.0])
            w_r, w_z = np.dot(wrist, forward), wrist[2] - self.d[0]
            for sign in (1, -1):
                elbow = sign * math.acos(cos_elbow)
                upper_arm = (math.atan2(w_z, w_r)
                             + math.atan2(a3 * math.sin(elbow), a2 + a3 * math.cos(elbow)))
                theta = [waist, -upper_arm, elbow]
                tool_elevation = math.atan2(axis[2], np.dot(axis, forward))
                theta.append(-math.pi / 2 - tool_elevation - theta[1] - theta[2])
                theta.append(0.0)
                q = [t - offset for t, offset in zip(theta, self.offsets)]
                # The faceplate turns the tool about its axis one to one.
                q[4] += _wrap(pose.roll - self._roll_of(self._flange(q), radial))
                q, problem = self._within_limits(q, near)
                if problem:
                    out_of_range.append(problem)
                else:
                    solutions.append(q)

        if not solutions:
            raise Unreachable('Out of the joint limits: ' + out_of_range[0] + '.')
        best = min(solutions, key=lambda q: self._time_from(near, q))
        self._check(best, pose)
        return best

    def _within_limits(self, q, near):
        """q with each joint on the turn closest to near and within its limits, or the first
        joint that cannot be."""
        result = []
        for name, angle, current, limit in zip(self.names, q, near, self.limits):
            turns = [angle + 2 * math.pi * k for k in range(-2, 3)]
            inside = [t for t in turns if limit.lower - _EPS <= t <= limit.upper + _EPS]
            if not inside:
                closest = min(turns, key=lambda t: abs(_wrap(t - current)))
                return None, (f'{name} would need {math.degrees(_wrap(closest)):.1f} deg, its '
                              f'range is {math.degrees(limit.lower):.1f} to '
                              f'{math.degrees(limit.upper):.1f} deg')
            result.append(min(inside, key=lambda t: abs(t - current)))
        return result, None

    def _time_from(self, near, q):
        """Time the slowest joint needs to go from near to q at full speed."""
        return max(abs(b - a) / limit.max_velocity for a, b, limit in zip(near, q, self.limits))

    def _check(self, q, pose):
        reached = self.fk(q)
        error = math.dist((reached.x, reached.y, reached.z), (pose.x, pose.y, pose.z))
        if error > 1e-6 or abs(_wrap(reached.pitch - pose.pitch)) > 1e-6 \
                or abs(_wrap(reached.roll - pose.roll)) > 1e-6:
            raise RuntimeError(f'Inverse kinematics missed the pose by {error * 1000:.3f} mm.')


def default_dh_config():
    """dh.yaml of the installed movemaster_description; without ROS, the one of this repo."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('movemaster_description')) / 'config' / 'dh.yaml'
    except Exception:  # No ROS, or the package is not installed.
        return (Path(__file__).resolve().parents[2] / 'movemaster_description' / 'config'
                / 'dh.yaml')


def default_joint_config():
    """joints.json of the installed movemaster_hardware; without ROS, the one of this repo."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('movemaster_hardware')) / 'config' / 'joints.json'
    except Exception:
        return (Path(__file__).resolve().parents[2] / 'movemaster_hardware' / 'config'
                / 'joints.json')
