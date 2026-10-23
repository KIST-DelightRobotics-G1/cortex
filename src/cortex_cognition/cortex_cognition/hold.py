"""hold — is the object we picked still in the gripper? Pure logic, no rclpy.

The detector judges what it sees in the gripper region and publishes it
(cortex_msgs/GripperPresence) without knowing the plan. The orchestrator knows
what it should be holding: `holding`, set when a pick-type step reports DONE.
This module joins the two; the executor decides what a mismatch means.

    GripperView            latest GripperPresence as plain data + its receive time
    view.state(key, now)   (PRESENT | ABSENT | UNKNOWN | UNSUPPORTED, detail)
    HoldStats              time share of each state while holding — the log that
                           measures wrist recognition with a real grasp

docs/gripper_presence.md is the contract with the detector side.
"""

from __future__ import annotations

PRESENT, ABSENT, UNKNOWN, UNSUPPORTED = 'PRESENT', 'ABSENT', 'UNKNOWN', 'UNSUPPORTED'
STATE_NAME = {0: UNKNOWN, 1: PRESENT, 2: ABSENT}     # ObjectPresence.state

# Steps that take an object into the hand / let it go (first arg = the object).
PICKS = ('pick', 'take_out', 'receive')
PUTS = ('place', 'put_in', 'handover')


class GripperView:
    """The latest GripperPresence. A message older than timeout_s means the detector
    stopped talking — every key reads UNKNOWN, never the last state it sent."""

    def __init__(self, timeout_s: float = 1.0) -> None:
        self.timeout_s = timeout_s
        self.t_recv = None
        self.camera_ok = False
        self.objects: dict = {}                       # key -> (state, confidence)

    def update(self, now: float, camera_ok: bool, objects: dict) -> None:
        self.t_recv, self.camera_ok, self.objects = now, bool(camera_ok), dict(objects)

    def state(self, key: str, now: float) -> tuple:
        if self.t_recv is None:
            return UNKNOWN, 'no_message'
        if now - self.t_recv > self.timeout_s:
            return UNKNOWN, 'stale'
        if not self.camera_ok:
            return UNKNOWN, 'camera_not_ok'
        if key not in self.objects:
            return UNSUPPORTED, 'not_judged'          # the detector does not judge this object
        state, confidence = self.objects[key]
        return state, f'{confidence:.2f}'


class HoldStats:
    """Seconds spent in each state for one holding window, sampled by the caller.
    A sample closes the time since the previous one under the previous state."""

    def __init__(self, key: str, now: float) -> None:
        self.key = key
        self.t0 = self.t_last = now
        self.last = None
        self.seconds = {PRESENT: 0.0, ABSENT: 0.0, UNKNOWN: 0.0}
        self.changes = 0

    def sample(self, now: float, state: str) -> bool:
        """Record state at now. True for the first sample and whenever the state changes."""
        if self.last in self.seconds:
            self.seconds[self.last] += max(0.0, now - self.t_last)
        first = self.last is None
        changed = not first and state != self.last
        self.changes += changed
        self.last, self.t_last = state, now
        return first or changed

    def summary(self, now: float) -> dict:
        if self.last is not None:
            self.sample(now, self.last)
        total = sum(self.seconds.values())
        share = {s.lower(): round(v / total, 3) if total > 0 else 0.0
                 for s, v in self.seconds.items()}
        return dict(object=self.key, duration_s=round(now - self.t0, 2), changes=self.changes,
                    **share)
