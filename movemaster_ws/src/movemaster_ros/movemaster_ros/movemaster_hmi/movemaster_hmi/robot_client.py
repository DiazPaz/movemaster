"""The HMI's API to the robot: everything the screens ask of the arm and everything they are
told about it goes through a RobotClient. Angles are degrees with Home at 0, one per joint in
the order of joints.json.

    client = robot_client.connect(joints, jog_axes)
    client.positions_changed.connect(...)   # and the other signals
    client.start()
    client.move_to([0, 30, -45, 0, 0], speed_percent=25)
    client.run_program(steps, speed_percent=25)   # steps as in movemaster_hmi.program
    client.stop()

RobotClient itself is the HMI without a robot: it stays 'offline' and refuses every command.
connect() returns the ROS 2 one (ros_robot_client) when ROS is installed.
"""

import math

from PySide6.QtCore import QObject, Signal

from movemaster_hmi.robot_model import check_speed, check_target

# Who commands the joints.
OFFLINE = 'offline'   # no controller_manager or no joint feedback
STOPPED = 'stopped'   # neither controller active: the hardware stopped on a fault
AUTO = 'auto'         # joint_trajectory_controller: the HMI can move the arm
MANUAL = 'manual'     # jog_controller: the pendant jogs it

_REFUSAL = {
    OFFLINE: 'The robot is offline: start the bringup.',
    STOPPED: 'The robot stopped on a fault: recover it before moving.',
    MANUAL: 'The pendant is in MANUAL: switch it to AUTO to move from the HMI.',
}


class RobotError(Exception):
    """A command the robot cannot take now; the message is for the operator."""


class RobotClient(QObject):
    positions_changed = Signal(list)      # measured joint angles, about 10 times a second
    gripper_changed = Signal(bool)        # True: closed
    selection_changed = Signal(int, int)  # joints the pendant's X and Y jog; 0: none
    mode_changed = Signal(str)            # OFFLINE, STOPPED, AUTO or MANUAL
    busy_changed = Signal(bool)           # a move or a program is running
    step_started = Signal(int)            # index of the program step that starts
    finished = Signal(bool, str)          # the move or program ended: ok, message

    def __init__(self, joints, jog_axes, parent=None):
        super().__init__(parent)
        self.joints = list(joints)
        self.jog_axes = jog_axes  # (X joint numbers, Y joint numbers) of the pendant
        self.mode = OFFLINE
        self.busy = False

    def start(self):
        """Connects to the robot; the signals start arriving."""

    def shutdown(self):
        """Stops what is running and disconnects."""

    def move_to(self, angles_deg, speed_percent):
        """Moves every joint to its angle; all start and stop together. finished reports it."""
        check_target(self.joints, angles_deg)
        check_speed(speed_percent)
        self._take('move', [float(angle) for angle in angles_deg], float(speed_percent))

    def go_home(self, speed_percent):
        self.move_to([0.0] * len(self.joints), speed_percent)

    def run_program(self, steps, speed_percent):
        """Runs waypoint, gripper and wait steps in order; step_started and finished report."""
        if not steps:
            raise ValueError('The program is empty.')
        check_speed(speed_percent)
        for number, step in enumerate(steps, 1):
            try:
                if step['type'] == 'waypoint':
                    check_target(self.joints, step['joints'])
                elif step['type'] == 'wait' and not (
                        math.isfinite(step['seconds']) and step['seconds'] >= 0):
                    raise ValueError('the wait must be 0 s or longer.')
            except ValueError as error:
                raise ValueError(f'Step {number}: {error}') from error
        self._take('program', [dict(step) for step in steps], float(speed_percent))

    def set_gripper(self, closed):
        if self.mode == OFFLINE:
            raise RobotError(_REFUSAL[OFFLINE])
        self._submit('gripper', bool(closed))

    def stop(self):
        """Stops the move or program in progress; the arm holds where it is."""
        self._submit('stop')

    def _take(self, *command):
        """A command that needs the arm in AUTO and free."""
        if self.mode != AUTO:
            raise RobotError(_REFUSAL[self.mode])
        if self.busy:
            raise RobotError('The robot is already moving: stop it first.')
        self._set_busy(True)
        self._submit(*command)

    def _set_busy(self, busy):
        self.busy = busy
        self.busy_changed.emit(busy)

    def _submit(self, *command):
        """Hands a command to whoever carries it out; nobody does without a robot."""


def connect(joints, jog_axes):
    """The client for this machine: ROS 2 if it is installed, else the offline one."""
    try:
        from movemaster_hmi.ros_robot_client import RosRobotClient
    except ImportError as error:
        print(f'[ROBOT] ROS 2 not available ({error}): the HMI runs without a robot.')
        return RobotClient(joints, jog_axes)
    return RosRobotClient(joints, jog_axes)
