"""The pendant's joystick button opens and closes the gripper; without ROS so it can be tested
alone.

The pendant toggles its gripper flag at each press: active closes the gripper (it grips),
inactive opens it. Only a change commands: the first frame after the pendant connects, or
comes back after going silent, is just the reference, so connecting it never moves the
gripper. The command is the opening of gripper_controller: 0 closed, 1 open.

The gripper can also be moved from elsewhere (the HMI), so the text pendant is told the real
state and sets its flag to it (pendant_text.format_gripper). A change that only makes the flag
agree with the real state is that sync, not a press: it commands nothing.
"""

import math

CLOSED = 0.0
OPEN = 1.0


def is_closed(opening):
    """True or False from the reported opening; None while there is none (NaN)."""
    if opening is None or math.isnan(opening):
        return None
    return opening < (OPEN + CLOSED) / 2


class GripperButton:

    def __init__(self):
        self.flag = None  # gripper flag of the last fresh frame; None: no reference yet

    def update(self, frame, closed=None):
        """Opening to command now, or None. closed: the real state, None if unknown."""
        if frame is None:
            self.flag = None
            return None
        previous, self.flag = self.flag, frame.gripper
        if previous is None or previous == frame.gripper or frame.gripper == closed:
            return None
        return CLOSED if frame.gripper else OPEN
