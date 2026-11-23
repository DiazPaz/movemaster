from movemaster_hmi.program import ProgramRunner

PROGRAM = [{'type': 'waypoint', 'joints': [10.0, 20.0]},
           {'type': 'gripper', 'closed': True},
           {'type': 'wait', 'seconds': 1.5}]


class Robot:
    """Records what the runner starts; the test ends each step."""

    def __init__(self):
        self.started = []

    def move(self, joints):
        self.started.append(('move', joints))

    def gripper(self, closed):
        self.started.append(('gripper', closed))

    def wait(self, seconds):
        self.started.append(('wait', seconds))


def runner():
    robot, steps, finished = Robot(), [], []
    return ProgramRunner(robot, steps.append, lambda *end: finished.append(end)), robot, steps, \
        finished


def test_runs_one_step_at_a_time_in_order():
    program, robot, steps, finished = runner()
    program.start(PROGRAM)
    assert robot.started == [('move', [10.0, 20.0])] and steps == [0]
    program.step_done(True)
    assert robot.started[-1] == ('gripper', True) and steps == [0, 1]
    program.step_done(True)
    assert robot.started[-1] == ('wait', 1.5) and program.running
    program.step_done(True)
    assert finished == [(True, 'Program finished.')] and not program.running


def test_a_failed_step_ends_the_program():
    program, robot, _, finished = runner()
    program.start(PROGRAM)
    program.step_done(False, 'the controller aborted the move.')
    assert finished == [(False, 'Step 1: the controller aborted the move.')]
    assert len(robot.started) == 1 and not program.running


def test_stop_forgets_the_rest():
    program, robot, _, finished = runner()
    program.start(PROGRAM)
    program.stop()
    program.step_done(True)  # the stopped step reports late: ignored
    assert finished == [(False, 'Program stopped.')] and len(robot.started) == 1


def test_later_edits_do_not_change_the_running_program():
    program, robot, _, _ = runner()
    steps = [dict(step) for step in PROGRAM]
    program.start(steps)
    steps[1]['closed'] = False
    program.step_done(True)
    assert robot.started[-1] == ('gripper', True)


def test_an_unknown_step_fails():
    program, _, _, finished = runner()
    program.start([{'type': 'dance'}])
    assert finished == [(False, "Step 1: unknown step type 'dance'")]
