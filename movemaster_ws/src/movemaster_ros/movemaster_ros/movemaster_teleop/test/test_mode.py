from movemaster_teleop.jog import Jogger, JointLimits
from movemaster_teleop.mode import ControllerSelector, JOG, TRAJECTORY
from movemaster_teleop.pendant_frame import PendantFrame

DT = 0.02
MANUAL = PendantFrame(manual=True)
AUTO = PendantFrame(manual=False)


def test_nothing_is_switched_before_the_controllers_are_listed():
    # At startup the switch may already be in MANUAL while the spawners still load the
    # controllers: asking then would be refused.
    selector = ControllerSelector(retry_s=1.0)
    assert selector.request(MANUAL, False, DT) is None
    selector.seen({TRAJECTORY}, selector.generation)
    assert selector.request(MANUAL, False, DT) == (JOG, TRAJECTORY)


def test_manual_hands_the_joints_to_the_jog_and_auto_gives_them_back():
    selector = ControllerSelector(retry_s=1.0, active=TRAJECTORY)
    assert selector.request(AUTO, False, DT) is None  # already in AUTO
    assert selector.request(MANUAL, False, DT) == (JOG, TRAJECTORY)
    assert selector.request(MANUAL, False, DT) is None  # one switch at a time
    selector.finished(ok=True)
    assert selector.active == JOG
    assert selector.request(MANUAL, False, DT) is None
    assert selector.request(AUTO, False, DT) == (TRAJECTORY, JOG)
    selector.finished(ok=True)
    assert selector.active == TRAJECTORY


def test_a_silent_pendant_switches_nothing():
    selector = ControllerSelector(retry_s=1.0, active=JOG)
    assert selector.request(None, False, DT) is None
    assert selector.active == JOG


def test_manual_waits_for_a_running_trajectory():
    selector = ControllerSelector(retry_s=1.0, active=TRAJECTORY)
    assert selector.request(MANUAL, True, DT) is None and selector.holding
    assert selector.request(MANUAL, False, DT) == (JOG, TRAJECTORY)
    assert not selector.holding


def test_a_refused_switch_is_retried_after_retry_s():
    selector = ControllerSelector(retry_s=0.1, active=TRAJECTORY)
    assert selector.request(MANUAL, False, DT) == (JOG, TRAJECTORY)
    selector.finished(ok=False)
    assert selector.active == TRAJECTORY
    assert selector.request(MANUAL, False, 0.05) is None
    assert selector.request(MANUAL, False, 0.06) == (JOG, TRAJECTORY)


def test_follows_the_controllers_the_manager_lists():
    selector = ControllerSelector(retry_s=1.0, active=TRAJECTORY)
    selector.seen({'joint_state_broadcaster', JOG}, selector.generation)
    assert selector.active == JOG  # switched by hand with ros2 control
    # A fault deactivates both: the pendant does not bring the arm back.
    selector.seen({'joint_state_broadcaster'}, selector.generation)
    assert selector.active is None
    assert selector.request(AUTO, False, DT) is None
    assert selector.request(MANUAL, False, DT) is None


def test_a_list_older_than_the_last_switch_is_ignored():
    selector = ControllerSelector(retry_s=1.0, active=TRAJECTORY)
    asked = selector.generation
    selector.request(MANUAL, False, DT)
    selector.seen({TRAJECTORY}, asked)  # answered while the switch was pending
    selector.finished(ok=True)
    selector.seen({TRAJECTORY}, asked)  # answered after it, but asked before
    assert selector.active == JOG


def test_release_restarts_the_jog_from_the_measured_positions():
    jog = Jogger(['joint_1'], {'joint_1': JointLimits(-1.0, 1.0, 0.2)}, x_joints=[1],
                 y_joints=[], deadband=0.1, max_lead=0.05, idle_timeout=10.0)
    frame = PendantFrame(x_joint=1, x=1000)
    jog.update(frame, {'joint_1': 0.0}, DT)
    jog.release()
    # The trajectory controller moved the joint meanwhile: no jump back to the old target.
    assert jog.update(frame, {'joint_1': 0.5}, DT)[0] > 0.5
