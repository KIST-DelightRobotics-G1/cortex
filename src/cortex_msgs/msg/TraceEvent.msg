# Human-readable "thought flow" for the display (gui_bridge_node). Published by the
# node that makes the decision, at the moment it makes it. Carries no control
# semantics — every consumer of this topic is a viewer.

std_msgs/Header header

string plan_id

uint8 HEARD       = 0   # utterance received                    title = transcript
uint8 THINKING    = 1   # PlanRequest sent to the LLM
uint8 PLAN_LINE   = 2   # a validated subtask line arrived      title = Korean action name
uint8 PLAN_END    = 3   # plan complete                         index = count
uint8 REPLY       = 4   # chat / none / confirm answer          title = say
uint8 STEP_START  = 5   # SubtaskCmd sent                       title = say
uint8 STEP_DONE   = 6
uint8 STEP_FAILED = 7   #                                       title = reason
uint8 GROUND      = 8   # detector check                        title = "냉장고 문 확인됨" / "... 보이지 않음"
uint8 CANCEL      = 9   # plan abandoned                        title = reason
uint8 PLAN_DONE   = 10
uint8 NOTE        = 11  # anything else worth showing
uint8  kind

int16  index           # subtask index when the event is about one step, else -1
string title           # one line, Korean, for the screen
string body            # optional detail (raw JSON line, validator message, ...) — debug view only
