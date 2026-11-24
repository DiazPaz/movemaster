"""The HMI's API to the robot: everything the screens ask of the arm and everything they are
told about it goes through a RobotClient. Angles are degrees with Home at 0, one per joint in
the order of joints.json. A pose of the tool tip (tcp) is [X, Y, Z, PITCH, ROLL]: mm in
base_link and degrees, as in movemaster_kinematics.Pose.

    client = robot_client.connect(joints, jog_axes, arm)
    client.positions_changed.connect(...)   # and the other signals
    client.start()
    client.move_to([0, 30, -45, 0, 0], speed_percent=25)
    client.move_to_pose([250, 0, 200, -90, 0], speed_percent=25)
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

    def __init__(self, joints, jog_axes, arm=None, parent=None):
        super().__init__(parent)
        self.joints = list(joints)
        self.jog_axes = jog_axes  # (X joint numbers, Y joint numbers) of the pendant
        self.arm = arm            # movemaster_kinematics.Arm; None: poses are not available
        self.mode = OFFLINE
        self.busy = False
        self.positions = None     # last measured angles, in degrees
        self.positions_changed.connect(self._remember_positions)

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

    def pose_of(self, angles_deg):
        """[X, Y, Z, PITCH, ROLL] of the tcp with the joints at angles_deg."""
        pose = self._kinematics().fk([math.radians(angle) for angle in angles_deg])
        return [pose.x * 1000, pose.y * 1000, pose.z * 1000,
                math.degrees(pose.pitch), math.degrees(pose.roll)]

    def joints_for(self, pose, near_deg=None):
        """Joint angles that put the tcp at pose, within the limits; of the solutions, the one
        closest to near_deg (default: where the arm is). ValueError says why there is none."""
        from movemaster_kinematics import Pose
        if len(pose) != 5 or not all(math.isfinite(value) for value in pose):
            raise ValueError('A pose is X, Y, Z (mm), PITCH and ROLL (degrees).')
        x, y, z, pitch, roll = pose
        near = near_deg if near_deg is not None else self.positions
        q = self._kinematics().ik(
            Pose(x / 1000, y / 1000, z / 1000, math.radians(pitch), math.radians(roll)),
            [math.radians(angle) for angle in near] if near is not None else None)
        return [math.degrees(angle) for angle in q]

    def move_to_pose(self, pose, speed_percent):
        """Moves the tcp to pose, joint by joint like move_to: all start and stop together."""
        self.move_to(self.joints_for(pose), speed_percent)

    def set_tool_length(self, length_mm):
        """Another tool for this session: from the flange (tool0) to the tcp, in mm. What
        stays is tool.length_mm in movemaster_description's dh.yaml."""
        self.arm = self._kinematics().with_tool_length(length_mm / 1000)

    def tool_length(self):
        """From the flange to the tcp, in mm."""
        return self._kinematics().tool_length * 1000

    def run_program(self, steps, speed_percent):
        """Runs waypoint, gripper and wait steps in order; step_started and finished report."""
        if not steps:
            raise ValueError('The program is empty.')
        check_speed(speed_percent)
        steps = [dict(step) for step in steps]
        near = self.positions  # a pose step goes to the solution closest to the step before
        for number, step in enumerate(steps, 1):
            try:
                if step['type'] == 'pose':
                    step.update(type='waypoint', joints=self.joints_for(step.pop('pose'), near))
                if step['type'] == 'waypoint':
                    check_target(self.joints, step['joints'])
                    near = step['joints']
                elif step['type'] == 'wait' and not (
                        math.isfinite(step['seconds']) and step['seconds'] >= 0):
                    raise ValueError('the wait must be 0 s or longer.')
            except ValueError as error:
                raise ValueError(f'Step {number}: {error}') from error
        self._take('program', steps, float(speed_percent))

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

    def _kinematics(self):
        if self.arm is None:
            raise RobotError('Poses need movemaster_kinematics, which is not available.')
        return self.arm

    def _remember_positions(self, angles_deg):
        self.positions = list(angles_deg)

    def _set_busy(self, busy):
        self.busy = busy
        self.busy_changed.emit(busy)

    def _submit(self, *command):
        """Hands a command to whoever carries it out; nobody does without a robot."""


def connect(joints, jog_axes, arm=None):
    """The client for this machine: ROS 2 if it is installed, else the offline one."""
    try:
        from movemaster_hmi.ros_robot_client import RosRobotClient
    except ImportError as error:
        print(f'[ROBOT] ROS 2 not available ({error}): the HMI runs without a robot.')
        return RobotClient(joints, jog_axes, arm)
    return RosRobotClient(joints, jog_axes, arm)


def load_arm(joint_config):
    """The kinematics of the arm of joint_config, or None if movemaster_kinematics is missing."""
    try:
        from movemaster_kinematics import Arm
    except ImportError as error:
        print(f'[ROBOT] movemaster_kinematics not available ({error}): no poses.')
        return None
    return Arm.from_files(joint_config=joint_config)
