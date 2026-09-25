# orchestrator_node -> llm_node. One utterance, one plan. llm_node answers with a
# stream of PlanStep messages that all carry this plan_id.

string plan_id      # p-<orchestrator start epoch>-<4-digit seq>; issued by the orchestrator
string utterance    # final STT transcript, verbatim
# Robot state the planner needs to answer sensibly. One of:
#   idle
#   running:<action>            e.g. "running:move_to"
#   awaiting_confirm:<summary>  a confirm question is pending
string state

# 로봇이 기억하는 자기 상태. 연속 발화("테이블에 놔")를 풀려면 이 둘이 필요하다.
#   at       마지막으로 도착에 성공한 장소. 시작값 home(S1). 이동 실패·중단 시 unknown
#   holding  지금 들고 있는 물건. 시작값 none
string at
string holding
