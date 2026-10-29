"""Joint jogging by moving position targets, without ROS so it can be tested alone.

Each cycle a jogged joint's target advances speed * dt, where speed is proportional to the
joystick. The SPARK MAX keeps following its target in position control, so a released
joystick stops the target and the joint holds there, also against gravity.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class JointLimits:
    lower: float      # rad
    upper: float      # rad
    max_speed: float  # rad/s at full joystick


def shape(value: float, deadband: float) -> float:
    """Joystick in -1..1 to speed fraction: zero inside the deadband, then quadratic."""
    magnitude = abs(value)
    if magnitude <= deadband:
        return 0.0
    fraction = min(1.0, (magnitude - deadband) / (1.0 - deadband))
    return math.copysign(fraction * fraction, value)


class Jogger:
    """Holds the position target of every joint while a jog session is open.

    A session opens when the joystick moves a selected joint: all targets start from the
    measured positions, so the arm never jumps to stale targets (for example, after the
    trajectory controller moved it). The session closes after idle_timeout without motion;
    the jog controller keeps holding its last command.
    """

    def __init__(self, joints, limits, x_joints, y_joints, deadband, max_lead, idle_timeout):
        self.joints = list(joints)
        self.limits = limits
        self.x_joints = set(x_joints)
        self.y_joints = set(y_joints)
        self.deadband = deadband
        self.max_lead = max_lead
        self.idle_timeout = idle_timeout
        self.targets = None
        self._idle = 0.0

    def release(self):
        """Ends the jog session: the next one starts from the measured positions."""
        self.targets = None
        self._idle = 0.0

    def speeds(self, frame):
        """Commanded speed (rad/s) of each jogged joint; empty without a frame or motion."""
        if frame is None:
            return {}
        result = {}
        for number, value, axis_joints in ((frame.x_joint, frame.x, self.x_joints),
                                           (frame.y_joint, frame.y, self.y_joints)):
            name = f'joint_{number}'
            fraction = shape(value / 1000.0, self.deadband)
            if number in axis_joints and name in self.limits and fraction:
                result[name] = fraction * self.limits[name].max_speed
        return result

    def update(self, frame, measured, dt):
        """Returns the targets to publish this cycle, or None to publish nothing."""
        speeds = self.speeds(frame)
        if not speeds:
            self._idle += dt
            if self._idle >= self.idle_timeout:
                self.targets = None
            return None
        self._idle = 0.0
        if self.targets is None:
            start = [measured.get(name, math.nan) for name in self.joints]
            if not all(math.isfinite(value) for value in start):
                return None
            self.targets = start
        for name, speed in speeds.items():
            i = self.joints.index(name)
            limits = self.limits[name]
            target = self.targets[i] + speed * dt
            # Never more than max_lead ahead of the joint, so a blocked joint does not
            # jump forward when freed, and never past the calibrated limits.
            here = measured[name]
            target = min(max(target, here - self.max_lead), here + self.max_lead)
            self.targets[i] = min(max(target, limits.lower), limits.upper)
        return list(self.targets)
