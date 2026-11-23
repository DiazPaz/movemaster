"""Runs the steps of an HMI program in order, one at a time; without ROS or Qt so it can be
tested alone.

A step is a dict, as the HMI stores it:
    {'type': 'waypoint', 'joints': [degrees, ...]}
    {'type': 'gripper', 'closed': bool}
    {'type': 'wait', 'seconds': float}
"""


class ProgramRunner:
    """Starts each step through `robot` and waits for step_done() before the next one.

    robot.move(joints_deg), robot.gripper(closed) and robot.wait(seconds) start a step; whoever
    carries it out reports its end with step_done(). A failed step ends the program.
    """

    def __init__(self, robot, on_step, on_finished):
        self.robot = robot
        self.on_step = on_step          # (index) a step starts
        self.on_finished = on_finished  # (ok, message) the program ended
        self.steps = []
        self.index = None               # step in progress; None: not running

    @property
    def running(self):
        return self.index is not None

    def start(self, steps):
        if self.running:
            raise RuntimeError('A program is already running.')
        self.steps = [dict(step) for step in steps]
        self.index = -1
        self._next()

    def step_done(self, ok, message=''):
        if not self.running:
            return
        if not ok:
            number = self.index + 1
            self.index = None
            self.on_finished(False, f'Step {number}: {message}')
            return
        self._next()

    def stop(self):
        """Forgets the program; the caller stops whatever the current step started."""
        if self.running:
            self.index = None
            self.on_finished(False, 'Program stopped.')

    def _next(self):
        self.index += 1
        if self.index == len(self.steps):
            self.index = None
            self.on_finished(True, 'Program finished.')
            return
        step = self.steps[self.index]
        self.on_step(self.index)
        if step['type'] == 'waypoint':
            self.robot.move(step['joints'])
        elif step['type'] == 'gripper':
            self.robot.gripper(step['closed'])
        elif step['type'] == 'wait':
            self.robot.wait(step['seconds'])
        else:
            self.step_done(False, f'unknown step type {step["type"]!r}')
