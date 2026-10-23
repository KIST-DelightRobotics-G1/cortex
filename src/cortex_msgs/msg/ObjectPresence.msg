# One object's state in the gripper region. The detector judges it; see docs/gripper_presence.md.
uint8 UNKNOWN=0          # no verdict: no or old frames, model error
uint8 PRESENT=1
uint8 ABSENT=2

string  key              # vocabulary key, the same keys as CheckTarget (e.g. "cucumber")
uint8   state
float32 confidence       # best confidence in the judging window (0 if none)
