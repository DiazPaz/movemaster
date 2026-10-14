"""jog_node: jogs MoveMaster joints from the teach pendant.

Reads pendant frames, moves the position targets of the selected joints (movemaster_teleop.jog)
and publishes them to the jog_controller, a JointGroupPositionController. It commands the arm
only while jog_controller is active:

    ros2 control switch_controllers --deactivate joint_trajectory_controller \
        --activate jog_controller
"""

import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from movemaster_teleop.jog import Jogger, JointLimits
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
        targets = self.jogger.update(frame, self.measured, dt)
        if targets is not None:
            self.publisher.publish(Float64MultiArray(data=targets))

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
