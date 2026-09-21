# orchestrator_node -> execution module (nav-planner or VLA). One message = one
# subtask. The module echoes plan_id / index / action back in every SubtaskState.
# See docs: Subtask_state_interface_spec v0.1.

std_msgs/Header header

string plan_id
uint16 index
string action          # move_to | open | close | pick | place | handover
string[] args          # signature order, vocabulary values only
string instruction     # VLA-only English instruction (orchestrator fills from actions.yaml); "" for nav
bool cancel            # true = stop the subtask identified by plan_id/index; other fields ignored
