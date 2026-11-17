from movemaster_teleop.gripper import CLOSED, GripperButton, OPEN
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
