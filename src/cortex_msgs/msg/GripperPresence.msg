# detector_node -> orchestrator_node: what is seen in the gripper region, per object key.
# Published on every state change and at least every 0.5 s. The detector does not know the
# plan; the orchestrator decides what an ABSENT means. Contract: docs/gripper_presence.md.
builtin_interfaces/Time stamp   # newest frame used
bool camera_ok                  # false: gripper camera or model unusable -> every key is UNKNOWN
ObjectPresence[] objects        # one entry per key the detector judges in the gripper region
