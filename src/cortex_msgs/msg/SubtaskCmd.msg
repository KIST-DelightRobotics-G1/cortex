# orchestrator_node -> execution module (nav-planner or VLA). One message = one
# subtask. The module echoes plan_id / index / action back in every SubtaskState.
# See docs: Subtask_state_interface_spec v0.1.
#
# 필드 구성은 nav(idl/kist_nav.idl) · VLA(idl/cortex_subtask.idl) 와 같아야 한다.
# 두 모듈 모두 Header 를 두지 않으므로 여기서도 두지 않는다 (2026-09 정합).

string plan_id
uint16 index
string action          # move_to | open | close | pick | place | handover
string[] args          # signature order, vocabulary values only
string instruction     # VLA-only English instruction (orchestrator fills from actions.yaml); "" for nav
bool cancel            # true = stop the subtask identified by plan_id/index; other fields ignored
