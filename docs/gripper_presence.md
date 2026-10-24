# Gripper presence — 운반 중 물체 놓침 감지

집은 물체가 놓기 전까지 그리퍼에 남아 있는지 본다. 판정은 detector, 의미는 orchestrator가 맡는다.

| 모듈 | 하는 일 | 모르는 것 |
|---|---|---|
| detector | 그리퍼 영역에 물체가 보이는지 판정해 `GripperPresence` 발행 | 계획, 단계, 들고 있어야 하는지 |
| orchestrator | 들고 있어야 하는 물체(`holding`)와 비교해 시작을 막거나 단계를 취소 | 카메라, 프레임 시각, 그리퍼 영역 |
| VLA · nav | 변경 없음. 감시에 걸리면 cancel만 받는다 | — |

## Detector가 지킬 계약

토픽 `/cortex/gripper_presence`, 메시지 `cortex_msgs/GripperPresence`.

- **언제:** 노드가 뜬 뒤 계속. 상태가 바뀌면 바로, 바뀌지 않아도 0.5초마다 발행한다(살아 있다는 신호).
- **`camera_ok`:** 그리퍼 카메라나 모델을 못 쓰면 `false`. 이때 orchestrator는 모든 물체를 UNKNOWN으로 본다.
- **`objects[]`:** 그리퍼 영역에서 판정하는 물체마다 한 줄. `key`는 `CheckTarget`과 같은 어휘 키(`cucumber`).
  판정하지 않는 물체는 넣지 않는다. 목록에 없는 물체는 감시하지 않는다.
- **`state`:**

| 상태 | 조건 (사양 v0.1) |
|---|---|
| `PRESENT` | 최근 0.6초에 프레임 3장 이상, 과반 검출, 마지막 프레임 검출 |
| `ABSENT` | 최근 1.0초 검출 비율 50% 미만, 마지막 프레임 미검출 |
| `UNKNOWN` | 프레임 없음 · 0.6초 넘게 지연 · 모델 오류 |

- 그리퍼 영역(ROI) 안의 검출만 인정한다. 냉장고 안에 남은 물체가 잡히지 않게 하기 위함이다.

## Orchestrator 동작

설정은 `cortex_params.yaml`의 `orchestrator_node`. 항상 켜져 있고 모드만 고른다.

| `held_monitor_mode` | 하는 일 |
|---|---|
| `log` (기본) | 판정과 기록만. 계획은 그대로 진행한다. 현장 인식률 확인용 |
| `enforce` | 아래처럼 단계를 막거나 멈춘다 |

- **감시 구간:** 집기(`pick` · `take_out` · `receive`)가 DONE이 된 뒤부터 같은 물체를 놓는 단계(`place` · `put_in` ·
  `handover`)가 시작될 때까지. 대상은 집기 동작의 첫 인자.
- **시작 전 확인 (enforce):** 감시 중이면 기존 `CheckTarget` 확인에 더해 그리퍼 PRESENT가 있어야 단계를 보낸다.
  재시도 간격과 제한 시간은 기존 값을 같이 쓴다. 못 채우면 "오이를 들고 있지 않아 작업을 중단합니다." → `holding = none`.
- **진행 중 (enforce):** ABSENT를 받으면 진행 중인 모듈에 cancel → 단계 실패(`object_lost`) → 계획 중단 →
  "오이를 놓쳤습니다. 작업을 중단합니다." → `holding = none`.
- **UNKNOWN:** 메시지가 `held_heartbeat_timeout_s`(1.0초) 넘게 없거나 `camera_ok=false`.
  enforce에서 `held_unknown: warn`이면 진행, `fail`이면 시작하지 않거나 단계를 멈춘다. 멈춰도 `holding`은 유지한다
  (안 보이는 것과 없는 것은 다르다).
- **감시하지 않는 경우:** 빈손일 때, 집기·놓기 단계 자체, 목록에 없는 물체.

## 로그

화면(`hold`)에는 필요한 때만 한 줄씩 남는다. 프레임마다, 상태가 바뀔 때마다 찍지 않는다.

| 이벤트 | 언제 | 내용 |
|---|---|---|
| `HOLD_CONFIG` | 노드 시작 시 한 번 | 모드, 토픽, 타임아웃 |
| `HOLD_START` | 집기 완료 (감시 구간 시작) | 대상, 그 순간 그리퍼 상태 |
| `HOLD_UNKNOWN` | 구간마다 이유별 한 번 | `no_message` · `stale` · `camera_not_ok` |
| `HOLD_LOST` | 단계마다 한 번 | ABSENT 판정. `stage`(precheck / running), `stopped`(enforce면 true) |
| `HOLD_STATS` | 구간 끝 (놓기 시작 · 놓침 · 다른 물체 집기) | PRESENT / ABSENT / UNKNOWN 시간 비율, `absent_runs`(ABSENT 횟수), `absent_max_s`(가장 긴 ABSENT), `unknown_why` |

`HOLD_STATS`가 실제로 쥐었을 때의 손목 인식률이다. 놓기 단계를 보내는 순간 구간이 끝나므로 일부러 놓는 동안의
ABSENT는 들어가지 않는다. 놓기가 실패하면 계획이 끝난 뒤 구간을 다시 연다(`HOLD_START reason=resumed`).

상태가 바뀔 때마다의 기록(`HOLD_STATE`)은 `hold_trace` 토픽이다. 화면에는 꺼져 있고 세션 jsonl에만 남는다
(`logging.yaml`의 `record: all`). detector가 깜빡이는지 볼 때는 `events`에 `hold_trace`를 더한다.

## Detector 없이 시험하기

```bash
python3 scripts/fake_gripper_presence.py --key cucumber --state p
```

`p` / `a` / `u` / `c`(camera_ok 전환) 입력으로 상태를 바꾼다. 스크립트를 끄면 1초 뒤 UNKNOWN이 된다.
