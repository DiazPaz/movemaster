"""The pendant's joystick button opens and closes the gripper; without ROS so it can be tested
alone.

The pendant toggles its gripper flag at each press: active closes the gripper (it grips),
inactive opens it. Only a change commands: the first frame after the pendant connects, or
comes back after going silent, is just the reference, so connecting it never moves the
gripper. The command is the opening of gripper_controller: 0 closed, 1 open.
"""

CLOSED = 0.0
OPEN = 1.0


class GripperButton:

    def __init__(self):
        self.flag = None  # gripper flag of the last fresh frame; None: no reference yet

    def update(self, frame):
        """Opening to command now, or None."""
        if frame is None:
            self.flag = None
            return None
        previous, self.flag = self.flag, frame.gripper
        if previous is None or previous == frame.gripper:
            return None
        return CLOSED if frame.gripper else OPEN
