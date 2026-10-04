# Selected YOLO26s model and bounded prechecks [SYS-REQ-44]

This profile connects **YOLO26s 3차 9/30 영상 기반 파인튜닝 (2차 모델 연속 학습)**
to the existing LLM-mode object-presence precheck. It does not verify door state,
reachability, graspability or task completion. Subtask/CheckTarget message fields
are unchanged. The VLA runner still owns DONE/FAILED; the next command waits for its IDLE.

Latest-main integration results: [local verification](detector_main_verification.md).

## Run on the workstation

Use the [standard deployment guide](yolo_deployment.md) for host/Docker installation.
The default `cortex.launch.py` and `cortex_params.yaml` now apply the v3 settings.
The selected model is still external:

`yolo26s_fridge_cucumber_continued.pt`

SHA-256: `3d275b84378da06dc9202b4d40962f49f8d88d18062357d1d167abab10d6f920`

```bash
ros2 launch cortex_bringup cortex.launch.py \
  model:=/absolute/path/yolo26s_fridge_cucumber_continued.pt device:=0 detector_only:=true
```

Omit detector_only to start the full Cortex graph. Camera/nav/VLA are external.
The default YAML model path matches Docker's /models/cortex read-only mount.
A full site YAML can be supplied via params_file. Model/device arguments override it.
The old cortex_yolo.launch.py remains a wrapper; profile names are not restricted
to a hard-coded list anymore. llm_demo.launch.py explicitly retains the always stub.
Missing weights stop standard launch; wrong hashes cannot authorize dispatch.
Default LLM backend remains dummy and VLA training sentences need configuration.

## Decision sequence

1. H.264 packets are decoded in order by one camera callback. Inference samples
   the latest decoded frame at a target 8 Hz; it does not run on every packet.
   The decoded frame's source PTS is retained even if the codec buffers/reorders
   frames. Missing decoded timestamps cannot count as fresh evidence. Source and
   receiver clocks must agree; a source clock reset requires restarting the node.
   Raw and compressed-image transports remain available.
2. Candidate boxes use confidence >= 0.25. Each frame contributes at most one vote
   per target. The recent 0.6-second window needs at least 3 distinct frames.
3. An object passes only if a strict majority has confidence >= 0.25 **and the
   latest processed frame also has a qualifying detection**. This prevents a prior
   positive majority from overriding an already processed negative frame.
4. Before checking objects, Cortex applies main's module-readiness gate. A
   non-IDLE destination enters WAIT_READY; a leftover RUNNING command is cancelled
   using the module's reported plan/index. The existing safe-stop timeout bounds
   this wait. After IDLE, Cortex starts a fresh PRECHECK and queries `CheckTarget`.
   If it cannot confirm the target, it retries at intervals of at least 0.1 seconds,
   with a 1.0-second deadline starting at PRECHECK entry. No VLA command is sent
   while waiting. If the module becomes busy during retries, Cortex returns to
   WAIT_READY and rechecks all objects after the next IDLE; prior hits are discarded.
5. A positive result before the deadline dispatches once. Persistent absence or
   unavailable input aborts the plan. Unknown/unsupported targets and invalid
   requests fail immediately. Fail-open is disabled.
6. A user stop or replacement plan can end PRECHECK without waiting for a module
   that has received no command. DONE received before dispatch cannot advance it.
7. After dispatch, DONE enters WAIT_IDLE; only the module's IDLE permits the next
   precheck/command. Missing IDLE aborts after the existing 1-second idle timeout.
   A stop during an active command sends cancel and waits for module IDLE. The
   PRECHECK-only exception applies before any command has been sent for that step.

The deadline uses the executor clock; timer granularity and a service call already
in progress can delay the failure notification. Responses arriving after the
deadline cannot authorize dispatch. Existing service timeout is 0.3 seconds.
This is a start-condition check, not a continuous stop controller during VLA motion.

`fridge_door` and `fridge` map to `refrigerator`; `cucumber` maps to `cucumber`.
`user` maps to unsupported `person`, so handover requiring a person is blocked.
A positive refrigerator result does not distinguish open from closed doors.

### Multi-target start condition for take_out

`actions.yaml` accepts a single argument slot or an AND list:

```yaml
precheck:
  open: object
  close: object
  pick: object
  take_out: [container, object]
```

`take_out(cucumber, fridge)` now checks `fridge` and `cucumber` on every attempt.
Under the v3 fail-closed profile, both must return `found=true` in that attempt
before one VLA command is sent. Earlier successful attempts are not remembered:
fridge-only followed by cucumber-only cannot satisfy the condition. All calls
share the same one-second step deadline; slow responses cannot extend it.
Timeout/failure messages identify the target still missing or unavailable.
Malformed slots, empty lists and missing required arguments cannot silently
remove a precheck. The other action requirements are unchanged; `put_in` still
checks its container. The explicit demo fail-open option retains its old meaning.

This reuses the existing `CheckTarget` service in sequence. Each call applies the
detector's fresh-frame window at query time; this is not an atomic, same-frame
co-occurrence test. The object/container spatial relationship, door state and
graspability are not established. The model and ROS wire schemas are unchanged.
Stop, replacement-plan and DONE-to-IDLE rules still apply. The demo `approach`
rewrite completes and returns IDLE before the two-target precheck is attempted.

Local regression coverage includes each single-object-only case, both visible,
alternating positives, latest-frame loss, unavailable/unsupported input, shared
timeout, stop, replacement plan, and the rewritten open/approach/take_out flow.
ROS Humble and live camera/VLA verification remain required on the workstation.

## VLA instruction configuration

Main's `config/vla_prompts.yaml` is loaded via the orchestrator's
`vla_prompts_path`. A populated action/argument entry is sent verbatim, including
for inserted approach/step_back actions; it does not bypass any precheck.
The shipped eight entries are empty and retain the existing template fallback
with a warning. Before live VLA testing, the VLA team must supply the exact
training/progress-head sentences described in that file. No sentences were
invented during this integration. DONE/FAILED still come from the VLA module.

## Comparison and reproducibility

Confidence 0.25 is retained from the report as a provisional baseline, not selected
by optimizing the reused test set. Compare 0.25/0.4 and majority-only/latest-hit
policies using the same saved predictions. This is exploratory analysis; validate
final settings on separate KIST execution records.

```bash
python3 -m pytest -q
python3 scripts/verify_detector_offline.py \
  --model /absolute/path/yolo26s_fridge_cucumber_continued.pt \
  --profile src/cortex_bringup/config/fridge_detector_v3.yaml \
  --video /absolute/path/test_video.mp4 \
  --device cpu --output /tmp/v3-inference.json
python3 scripts/replay_preconditions.py \
  --evidence /tmp/v3-inference.json \
  --profile src/cortex_bringup/config/fridge_detector_v3.yaml \
  --near-timing /absolute/path/continued_v3.json \
  --visibility /absolute/path/temporal_annotations.json \
  --output /tmp/v3-replay.json
```

Outputs must not already exist. The second script uses the reviewed near test
video: `near-timing` supplies original PTS and `visibility` the existing frame
intervals. It validates the selected weight hash and inference configuration.

The subtask replay uses real Cortex planner validation, presence policy and executor.
It injects the plan `open(fridge_door) -> pick(cucumber) -> close(fridge_door)`.
The installed `plan_rewrites.yaml` adds the current main's demo-only `approach`
and `step_back`, giving `open -> approach -> pick -> step_back -> close`.
These inserted actions retain main's existing behavior without an added object
precheck. Original open/pick/close still query refrigerator/cucumber/refrigerator.
VLA heartbeats, DONE and cleanup IDLE are **mocked**, with manual DONE times of
1.0 / 10.3 / 23.0 / 23.5 / 25.2 seconds and an IDLE 0.3 seconds after each DONE.
Separate scenarios request pick too early, omit DONE, omit IDLE after DONE, or stop
during PRECHECK. These are software integration checks, not successful robot trials
or estimated action durations. The rewriter can be disabled via
`plan_rewrites_path: ''` in the orchestrator configuration for deployments that do
not use those demo rules.

The presence comparison evaluates queries at completed inference timestamps,
roughly 8 Hz, against visibility at those timestamps. Counts differ from the
report's 105 images and 815 dense frames. An unavailable response cannot authorize.
Ambiguous visibility frames are excluded. The latest-frame requirement can increase
misses while reducing delayed disappearance; retries may recover, but do not
eliminate every missed opportunity. Between inference updates an earlier verdict
may remain available, and camera/ROS transport adds unmeasured delay.

ROS Humble build/generated-message tests, actual camera timing/QoS, live VLA
integration and RTX 4090 end-to-end latency remain to be measured.
