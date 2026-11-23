from movemaster_teleop import pendant_joy
from movemaster_teleop.pendant_frame import PendantFrame


def test_a_frame_survives_the_joy_message():
    sent = PendantFrame(manual=True, gripper=True, x_joint=5, y_joint=2, x=-1000, y=337)
    assert pendant_joy.axes(sent) == [-1.0, 0.337]
    assert pendant_joy.buttons(sent) == [1, 1, 5, 2]
    assert pendant_joy.frame(pendant_joy.axes(sent), pendant_joy.buttons(sent)) == sent


def test_nothing_selected_in_auto():
    idle = PendantFrame()
    assert pendant_joy.frame(pendant_joy.axes(idle), pendant_joy.buttons(idle)) == idle
