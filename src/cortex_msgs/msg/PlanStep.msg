# llm_node -> orchestrator_node. One NDJSON line from the LLM, already validated
# against actions.yaml. Streamed: the orchestrator starts step 0 before END arrives.

string plan_id

uint8 KIND_SUB   = 0   # a subtask: action + args (+ say)
uint8 KIND_END   = 1   # plan complete; `count` = number of subtasks
uint8 KIND_REPLY = 2   # no plan: chat / none / confirm — `say` is the whole answer
uint8 KIND_ERROR = 3   # llm_node gave up (timeout, API error, validation failure)
uint8  kind

uint16 index           # subtask order, 0-based (KIND_SUB); count for KIND_END
string action          # actions.yaml key, e.g. move_to (KIND_SUB)
string[] args          # signature-ordered vocabulary values (KIND_SUB)
string say             # Korean line to speak (SUB: on start; REPLY: the answer)
string reply_kind      # KIND_REPLY only: chat | none | confirm
string detail          # KIND_ERROR reason; otherwise ""
