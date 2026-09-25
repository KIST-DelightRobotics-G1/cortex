# execution module -> orchestrator_node. Published at 10 Hz at all times (IDLE too,
# as the liveness signal). RELIABLE, depth 10.
#   IDLE     no subtask; module is clean and ready for a new SubtaskCmd
#   RUNNING  the subtask identified below is executing
#   DONE     finished successfully — repeated exactly 3 times (300 ms), then IDLE
#   FAILED   finished unsuccessfully — same 3x rule; `detail` says why
# plan_id / index / action echo the SubtaskCmd being reported ("" / 0 / "" in IDLE).

# stamp_ns 는 발행 시각(epoch ns). 신선도(stale) 판정에 쓴다. VLA 의 IDL 과 같은 형태이며,
# nav 도 Header 대신 이 필드로 맞추기로 했다 (2026-09 정합).
int64 stamp_ns

string plan_id
uint16 index
string action

uint8 IDLE    = 0
uint8 RUNNING = 1
uint8 DONE    = 2
uint8 FAILED  = 3
uint8  status

float32 progress       # 0..1, debugging/GUI only — never used for the verdict
string  detail         # "cancelled", "unsupported: ...", "timeout", ...
