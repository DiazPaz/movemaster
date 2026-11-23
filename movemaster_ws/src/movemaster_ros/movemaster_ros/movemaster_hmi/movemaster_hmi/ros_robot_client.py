"""RobotClient over ROS 2, using only what the bringup already offers:

    /joint_states, /dynamic_joint_states      measured angles and gripper opening
    /pendant/joy                              what the pendant has selected (jog_node)
    /controller_manager/list_controllers      who commands the joints
    /joint_trajectory_controller/follow_joint_trajectory   moves
    /gripper_controller/commands              gripper: 0 closed, 1 open

The node spins in its own thread. Commands from the HMI cross to it through a queue and what it
learns goes back as Qt signals, so every ROS call happens in the ROS thread and every widget
is touched in the Qt thread.
"""

import math
import queue
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListControllers
from PySide6.QtCore import QCoreApplication, Signal
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from movemaster_hmi.program import ProgramRunner
from movemaster_hmi.robot_client import AUTO, MANUAL, OFFLINE, RobotClient, STOPPED
from movemaster_hmi.robot_model import move_duration
from movemaster_teleop import pendant_joy
from movemaster_teleop.gripper import CLOSED, OPEN
from movemaster_teleop.mode import JOG, TRAJECTORY

TRAJECTORY_ACTION = f'/{TRAJECTORY}/follow_joint_trajectory'
FEEDBACK_PERIOD_S = 0.1    # positions_changed
FEEDBACK_TIMEOUT_S = 1.0   # no /joint_states for this long: offline
MODE_PERIOD_S = 0.5        # how often the controller_manager is asked who commands
COMMAND_PERIOD_S = 0.02    # how often the HMI's commands are taken
GRIPPER_SETTLE_S = 1.0     # a program waits this long after commanding the gripper


class RosRobotClient(RobotClient):
    ros_stopped = Signal()  # ROS shut down from outside (Ctrl+C in the launch): the HMI closes

    def __init__(self, joints, jog_axes, parent=None):
        super().__init__(joints, jog_axes, parent)
        self._names = [joint.name for joint in self.joints]
        self._commands = queue.SimpleQueue()
        self._measured = {}        # joint name -> rad
        self._measured_at = None
        self._reported_at = 0.0
        self._selection = None
        self._gripper_closed = None
        self._listing = False
        self._move = None          # the move in progress: who is told when it ends, its goal
        self._timer = None         # one-shot timer of a wait or gripper step
        self._speed = 100.0
        self._closing = False
        steps = SimpleNamespace(move=self._program_move, gripper=self._program_gripper,
                                wait=self._program_wait)
        self._runner = ProgramRunner(steps, self.step_started.emit, self._finish)
        self.ros_stopped.connect(QCoreApplication.quit)

    def start(self):
        if not rclpy.ok():
            rclpy.init()
        node = self._node = Node('movemaster_hmi')
        self._trajectory = ActionClient(node, FollowJointTrajectory, TRAJECTORY_ACTION)
        self._controllers = node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self._gripper = node.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)
        node.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        node.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._on_dynamic_states, 10)
        node.create_subscription(Joy, pendant_joy.TOPIC, self._on_joy, 10)
        node.create_timer(MODE_PERIOD_S, self._refresh_mode)
        node.create_timer(COMMAND_PERIOD_S, self._take_commands)
        self._thread = threading.Thread(target=self._spin, name='ros', daemon=True)
        self._thread.start()

    def shutdown(self):
        self._closing = True
        self._commands.put(('stop',))
        time.sleep(2 * COMMAND_PERIOD_S)  # let the stop reach the controller
        rclpy.try_shutdown()
        self._thread.join(timeout=2.0)

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if not self._closing:
                self.ros_stopped.emit()

    def _submit(self, *command):
        self._commands.put(command)

    # From here on, everything runs in the ROS thread.

    def _take_commands(self):
        while True:
            try:
                kind, *arguments = self._commands.get_nowait()
            except queue.Empty:
                return
            if kind == 'move':
                self._start_move(*arguments, self._finish)
            elif kind == 'program':
                steps, self._speed = arguments
                self._runner.start(steps)
            elif kind == 'gripper':
                self._command_gripper(*arguments)
            elif kind == 'stop':
                self._stop()

    def _on_joint_states(self, msg):
        self._measured.update(zip(msg.name, msg.position))
        now = self._measured_at = time.monotonic()
        if now - self._reported_at >= FEEDBACK_PERIOD_S:
            angles = self._angles()
            if angles is not None:
                self._reported_at = now
                self.positions_changed.emit(angles)

    def _angles(self):
        """Measured angle of every joint in degrees, or None until all have reported."""
        if not all(name in self._measured for name in self._names):
            return None
        return [math.degrees(self._measured[name]) for name in self._names]

    def _on_dynamic_states(self, msg):
        for name, interfaces in zip(msg.joint_names, msg.interface_values):
            if name == 'gripper' and 'opening' in interfaces.interface_names:
                opening = interfaces.values[interfaces.interface_names.index('opening')]
                closed = opening < (OPEN + CLOSED) / 2
                if closed != self._gripper_closed:
                    self._gripper_closed = closed
                    self.gripper_changed.emit(closed)

    def _on_joy(self, msg):
        if len(msg.axes) < 2 or len(msg.buttons) < 4:
            return
        frame = pendant_joy.frame(msg.axes, msg.buttons)
        selection = (frame.x_joint, frame.y_joint)
        if selection != self._selection:
            self._selection = selection
            self.selection_changed.emit(*selection)

    def _refresh_mode(self):
        fresh = (self._measured_at is not None
                 and time.monotonic() - self._measured_at <= FEEDBACK_TIMEOUT_S)
        if not fresh or not self._controllers.service_is_ready():
            self._set_mode(OFFLINE)
        elif not self._listing:
            self._listing = True
            self._controllers.call_async(ListControllers.Request()).add_done_callback(
                self._on_controllers)

    def _on_controllers(self, future):
        self._listing = False
        response = future.result()
        if response is None:
            return
        active = {c.name for c in response.controller if c.state == 'active'}
        self._set_mode(MANUAL if JOG in active else AUTO if TRAJECTORY in active else STOPPED)

    def _set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.mode_changed.emit(mode)

    def _start_move(self, angles_deg, speed_percent, done):
        start = self._angles()
        if start is None or not self._trajectory.server_is_ready():
            done(False, 'The trajectory controller is not available.')
            return
        point = JointTrajectoryPoint(
            positions=[math.radians(angle) for angle in angles_deg],
            # At rest at the end: the controller joins both ends with a smooth curve.
            velocities=[0.0] * len(angles_deg),
            time_from_start=Duration(
                seconds=move_duration(self.joints, start, angles_deg, speed_percent)).to_msg())
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._names
        goal.trajectory.points = [point]
        # Each move is its own object: answers about a move that was stopped are recognized.
        move = self._move = SimpleNamespace(done=done, goal=None)
        self._trajectory.send_goal_async(goal).add_done_callback(
            lambda future: self._on_goal_response(move, future))

    def _on_goal_response(self, move, future):
        handle = future.result()
        if move is not self._move:  # stopped before the controller answered
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self._end_move(False, 'The trajectory controller rejected the move.')
            return
        move.goal = handle
        handle.get_result_async().add_done_callback(
            lambda result: self._on_result(move, result))

    def _on_result(self, move, future):
        if move is not self._move:
            return  # a move that was stopped
        response = future.result()
        if response is None:
            self._end_move(False, 'No result from the trajectory controller.')
        elif (response.status == GoalStatus.STATUS_SUCCEEDED
              and response.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self._end_move(True, 'Move finished.')
        else:
            self._end_move(False, response.result.error_string
                           or 'The trajectory controller aborted the move.')

    def _end_move(self, ok, message):
        move, self._move = self._move, None
        if move is not None:
            move.done(ok, message)

    def _command_gripper(self, closed):
        self._gripper.publish(Float64MultiArray(data=[CLOSED if closed else OPEN]))

    def _program_move(self, angles_deg):
        self._start_move(angles_deg, self._speed, self._runner.step_done)

    def _program_gripper(self, closed):
        self._command_gripper(closed)
        self._after(GRIPPER_SETTLE_S, lambda: self._runner.step_done(True))

    def _program_wait(self, seconds):
        self._after(seconds, lambda: self._runner.step_done(True))

    def _after(self, seconds, callback):
        def fire():
            self._cancel_timer()
            callback()
        self._timer = self._node.create_timer(max(seconds, 0.001), fire)

    def _cancel_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
            self._timer = None

    def _stop(self):
        if not self.busy:
            return
        self._cancel_timer()
        move, self._move = self._move, None
        if move is not None and move.goal is not None:
            move.goal.cancel_goal_async()  # the controller holds the arm where it is
        was_program = self._runner.running
        if was_program:
            self._runner.stop()
        else:
            self._finish(False, 'Move stopped.')

    def _finish(self, ok, message):
        self._set_busy(False)
        self.finished.emit(ok, message)
