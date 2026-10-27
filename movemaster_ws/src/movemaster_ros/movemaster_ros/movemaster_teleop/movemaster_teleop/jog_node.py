"""jog_node: jogs MoveMaster joints from the teach pendant.

Reads pendant frames, moves the position targets of the selected joints (movemaster_teleop.jog)
and publishes them to the jog_controller, a JointGroupPositionController. The pendant's MANUAL
switch picks the controller in command (movemaster_teleop.mode): MANUAL activates
jog_controller, AUTO gives the joints back to joint_trajectory_controller. With
select_controllers false, the controllers are switched by hand with ros2 control.
"""

import json
from pathlib import Path
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from movemaster_teleop.jog import Jogger, JointLimits
from movemaster_teleop.mode import ControllerSelector, JOG
from movemaster_teleop.pendant_link import SerialLink


def load_limits(path, max_speeds):
    """Joint order and limits from joints.json; jog speed capped at max_velocity_rad_s."""
    joints = json.loads(Path(path).read_text())['joints']
    limits = {}
    for name, joint in joints.items():
        number = int(name.rsplit('_', 1)[1])
        speed = max_speeds[number - 1] if number <= len(max_speeds) else 0.0
        limits[name] = JointLimits(joint['min_position_rad'], joint['max_position_rad'],
                                   min(speed, joint['max_velocity_rad_s']))
    return list(joints), limits


TRAJECTORY_STATUS = '/joint_trajectory_controller/follow_joint_trajectory/_action/status'
# Goals of the trajectory action that still move, or may move, the arm.
RUNNING = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)


def describe(controller):
    return {JOG: 'jog_controller in command (MANUAL).',
            None: 'No controller in command: the pendant switch does nothing until one is '
                  'active again.'}.get(controller, f'{controller} in command (AUTO).')


class JogNode(Node):

    def __init__(self):
        super().__init__('jog_node')
        default_config = str(Path(get_package_share_directory('movemaster_hardware'))
                             / 'config' / 'joints.json')
        p = {name: self.declare_parameter(name, value).value for name, value in {
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'joint_config': default_config,
            'rate_hz': 50.0,
            'x_joints': [1, 5],
            'y_joints': [2, 3, 4],
            'max_speed_rad_s': [0.25, 0.25, 0.25, 0.25, 0.25],
            'deadband': 0.12,
            'max_lead_rad': 0.05,
            'link_timeout_s': 0.2,
            'idle_timeout_s': 1.0,
            'select_controllers': True,
            'switch_retry_s': 2.0,
        }.items()}
        joints, limits = load_limits(p['joint_config'], p['max_speed_rad_s'])
        self.jogger = Jogger(joints, limits, p['x_joints'], p['y_joints'], p['deadband'],
                             p['max_lead_rad'], p['idle_timeout_s'])
        self.link = SerialLink(p['port'], p['baudrate'])
        self.link_timeout = p['link_timeout_s']
        self.measured = {}
        self.frame = None
        self.frame_at = None
        self.selection = None
        self.last_tick = time.monotonic()
        self.publisher = self.create_publisher(Float64MultiArray, '/jog_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.selector = None
        if p['select_controllers']:
            self.selector = ControllerSelector(p['switch_retry_s'])
            self.trajectory_running = False
            self.switch_client = self.create_client(
                SwitchController, '/controller_manager/switch_controller')
            self.list_client = self.create_client(
                ListControllers, '/controller_manager/list_controllers')
            self.create_subscription(
                GoalStatusArray, TRAJECTORY_STATUS, self.on_trajectory_status,
                qos_profile_action_status_default)
            self.create_timer(1.0, self.refresh_controllers)
        self.create_timer(1.0 / p['rate_hz'], self.tick)
        self.get_logger().info(
            f'Pendant on {p["port"]}; joints {", ".join(joints)}; '
            f'X jogs {p["x_joints"]}, Y jogs {p["y_joints"]}.')

    def on_joint_states(self, msg):
        self.measured.update(zip(msg.name, msg.position))

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(now - self.last_tick, 0.1), now
        was_connected = self.link.connected
        for frame in self.link.poll():
            self.frame, self.frame_at = frame, now
        if self.link.connected != was_connected:
            state = 'connected' if self.link.connected else 'disconnected'
            self.get_logger().info(f'Pendant {state}.')
        fresh = self.frame_at is not None and now - self.frame_at <= self.link_timeout
        frame = self.frame if fresh else None
        self.report_selection(frame)
        if self.selector is not None:
            self.select_controller(frame, dt)
            if self.selector.active != JOG or self.selector.pending:
                self.jogger.release()
                return
        targets = self.jogger.update(frame, self.measured, dt)
        if targets is not None:
            self.publisher.publish(Float64MultiArray(data=targets))

    def on_trajectory_status(self, msg):
        self.trajectory_running = any(goal.status in RUNNING for goal in msg.status_list)

    def select_controller(self, frame, dt):
        if not self.switch_client.service_is_ready():
            return
        was_holding = self.selector.holding
        switch = self.selector.request(frame, self.trajectory_running, dt)
        if self.selector.holding and not was_holding:
            self.get_logger().warn('MANUAL: the jog waits for the running trajectory to finish.')
        if switch is None:
            return
        activate, deactivate = switch
        request = SwitchController.Request(
            activate_controllers=[activate], deactivate_controllers=[deactivate],
            strictness=SwitchController.Request.STRICT)
        self.switch_client.call_async(request).add_done_callback(self.on_switched)

    def on_switched(self, future):
        response = future.result()
        target = self.selector.pending
        self.selector.finished(response is not None and response.ok)
        if self.selector.active == target:
            self.get_logger().info(describe(target))
        else:
            reason = getattr(response, 'message', '') if response is not None else ''
            self.get_logger().error(
                f'controller_manager refused to switch to {target}'
                f'{": " + reason if reason else ""}; retrying in {self.selector.retry_s:g} s.')

    def refresh_controllers(self):
        if self.selector.pending or not self.list_client.service_is_ready():
            return
        generation = self.selector.generation
        self.list_client.call_async(ListControllers.Request()).add_done_callback(
            lambda future: self.on_listed(future, generation))

    def on_listed(self, future, generation):
        response = future.result()
        if response is None:
            return
        before = self.selector.active
        self.selector.seen({c.name for c in response.controller if c.state == 'active'},
                           generation)
        if self.selector.active != before:
            self.get_logger().info(describe(self.selector.active))

    def report_selection(self, frame):
        selection = (frame.x_joint, frame.y_joint) if frame else None
        if selection == self.selection:
            return
        self.selection = selection
        if frame is None:
            self.get_logger().warn('No pendant frames: jogging stopped.')
            return
        names = [f'joint_{n}' + ('' if f'joint_{n}' in self.jogger.limits else ' (not configured)')
                 for n in selection if n]
        self.get_logger().info(f'Selected: {", ".join(names) or "none"}.')


def main():
    rclpy.init()
    node = JogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.link.close()
        node.destroy_node()
        rclpy.try_shutdown()
