"""The pendant on /pendant/joy (sensor_msgs/Joy), for whoever shows or uses what the operator
does without owning the serial port, like the HMI. jog_node owns the port and publishes each
fresh frame; no message means no pendant. Without ROS: only the layout of the message.

    axes     [x, y]                               joystick, -1..1
    buttons  [manual, gripper, x_joint, y_joint]  switch and gripper flag (0/1), selected joints
                                                  (0 = none)
"""

from movemaster_teleop.pendant_frame import FULL_SCALE, PendantFrame

TOPIC = '/pendant/joy'


def axes(frame):
    return [frame.x / FULL_SCALE, frame.y / FULL_SCALE]


def buttons(frame):
    return [int(frame.manual), int(frame.gripper), frame.x_joint, frame.y_joint]


def frame(joy_axes, joy_buttons):
    """The PendantFrame a message carries (seq is not sent)."""
    manual, gripper, x_joint, y_joint = joy_buttons[:4]
    return PendantFrame(manual=bool(manual), gripper=bool(gripper), x_joint=x_joint,
                        y_joint=y_joint, x=round(joy_axes[0] * FULL_SCALE),
                        y=round(joy_axes[1] * FULL_SCALE))
