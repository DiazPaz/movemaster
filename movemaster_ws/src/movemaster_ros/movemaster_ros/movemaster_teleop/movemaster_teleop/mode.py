"""Which controller commands the arm, chosen with the pendant's MANUAL switch; without ROS
so it can be tested alone.

MANUAL hands the joints to jog_controller and AUTO hands them back to
joint_trajectory_controller. Only a fresh pendant frame chooses: if the pendant goes
silent the active controller stays (the jog stops and the arm holds), so losing the
pendant never hands the arm to automatic motion. With neither controller active, as after
a hardware fault, nothing is switched: bringing the arm back is the operator's decision.
"""

TRAJECTORY = 'joint_trajectory_controller'
JOG = 'jog_controller'


class ControllerSelector:
    """Asks for one switch at a time and waits retry_s after a failed one.

    The active controller starts unknown (None) and comes from the controller_manager's list
    (seen()): at startup the spawners may not have loaded the controllers yet.
    """

    def __init__(self, retry_s, active=None):
        self.active = active    # controller in command, as last seen; None: neither or unknown
        self.pending = None     # controller being switched to
        self.holding = False    # MANUAL waits for a running trajectory to finish
        self.generation = 0     # switches finished, to drop controller lists older than one
        self.retry_s = retry_s
        self._wait = 0.0

    def request(self, frame, trajectory_running, dt):
        """(activate, deactivate) to ask the controller_manager for now, or None."""
        self._wait = max(0.0, self._wait - dt)
        wanted = None if frame is None else (JOG if frame.manual else TRAJECTORY)
        # A trajectory that is running finishes before the jog takes the joints.
        self.holding = wanted == JOG and self.active == TRAJECTORY and trajectory_running
        if (wanted is None or self.active is None or wanted == self.active or self.holding
                or self.pending or self._wait > 0):
            return None
        self.pending = wanted
        return wanted, self.active

    def finished(self, ok):
        """The answer to the last request."""
        if ok:
            self.active = self.pending
        else:
            self._wait = self.retry_s
        self.pending = None
        self.generation += 1

    def seen(self, active_controllers, generation):
        """Controllers the controller_manager listed as active, asked at generation.

        Keeps the selection right when someone switches with ros2 control or a fault
        deactivates both; a list asked before the last switch finished is stale.
        """
        if self.pending or generation != self.generation:
            return
        self.active = next((c for c in (JOG, TRAJECTORY) if c in active_controllers), None)
