# orchestrator_node -> llm_node. One utterance, one plan. llm_node answers with a
# stream of PlanStep messages that all carry this plan_id.

string plan_id      # p-<orchestrator start epoch>-<4-digit seq>; issued by the orchestrator
string utterance    # final STT transcript, verbatim
# Robot state the planner needs to answer sensibly. One of:
#   idle
#   running:<action>            e.g. "running:move_to"
#   awaiting_confirm:<summary>  a confirm question is pending
string state
