from movemaster_teleop.gripper import CLOSED, GripperButton, is_closed, OPEN
from movemaster_teleop.pendant_frame import PendantFrame

ACTIVE = PendantFrame(gripper=True)
INACTIVE = PendantFrame(gripper=False)


def test_each_press_closes_or_opens():
    button = GripperButton()
    assert button.update(INACTIVE) is None  # the reference
    assert button.update(INACTIVE) is None
    assert button.update(ACTIVE) == CLOSED
    assert button.update(ACTIVE) is None
    assert button.update(INACTIVE) == OPEN


def test_connecting_the_pendant_moves_nothing():
    # A pendant that powers up, or comes back, with its flag set does not close the gripper.
    button = GripperButton()
    assert button.update(ACTIVE) is None
    assert button.update(None) is None  # silent pendant
    assert button.update(INACTIVE) is None
    assert button.update(ACTIVE) == CLOSED


def test_a_sync_from_the_real_state_commands_nothing():
    # The HMI closed the gripper; the pendant, told so, sets its flag: that is not a press.
    button = GripperButton()
    assert button.update(INACTIVE, closed=False) is None  # the reference
    assert button.update(ACTIVE, closed=True) is None  # the sync after the HMI closed it
    assert button.update(INACTIVE, closed=True) == OPEN  # the next press opens it


def test_a_press_the_robot_has_not_reported_yet_still_commands():
    button = GripperButton()
    assert button.update(INACTIVE, closed=False) is None
    assert button.update(ACTIVE, closed=False) == CLOSED
    assert button.update(ACTIVE, closed=True) is None  # the robot reports it closed


def test_a_failed_command_brings_the_flag_back_without_commanding():
    # The gripper did not close (controller inactive): the pendant is told it is open.
    button = GripperButton()
    assert button.update(INACTIVE, closed=False) is None
    assert button.update(ACTIVE, closed=False) == CLOSED
    assert button.update(INACTIVE, closed=False) is None


def test_the_reported_opening():
    assert is_closed(0.0) is True and is_closed(0.3) is True
    assert is_closed(1.0) is False and is_closed(0.7) is False
    assert is_closed(float('nan')) is None and is_closed(None) is None
