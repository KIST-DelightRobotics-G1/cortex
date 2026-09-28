#!/usr/bin/env bash
# Inside the image: run the offline loop and wait for one plan to finish.
# Used by .github/workflows/docker.yml.
set -u

ros2 launch cortex_bringup llm_demo.launch.py > /tmp/launch.log 2>&1 &
sleep 10

timeout 70 ros2 topic echo /cortex/trace cortex_msgs/msg/TraceEvent > /tmp/trace.log 2>&1 &
sleep 3
# The dummy LLM only replays actions.yaml's few-shot examples; this one is four
# VLA steps with no move_to, so every argument passes the capability check.
ros2 topic pub -1 /cortex/stt/transcript std_msgs/msg/String "{data: '이 컵 찬장에 넣어줘'}"

# kind 10 = PLAN_DONE
for _ in $(seq 1 60); do
    grep -q "kind: 10" /tmp/trace.log && break
    sleep 1
done

echo "--- trace titles"
grep "title:" /tmp/trace.log || true
if grep -q "kind: 10" /tmp/trace.log; then
    echo "PLAN_DONE seen"
    exit 0
fi
echo "--- no PLAN_DONE; launch log"
tail -80 /tmp/launch.log
exit 1
