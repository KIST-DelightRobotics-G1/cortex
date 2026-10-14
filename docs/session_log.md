# 발화 세션 로그

발화 명령 하나(전사 → 계획 → 실행 → 종료)를 파일 하나로 남긴다. `session_recorder_node` 가
`/cortex/events` 를 받아 세션별로 묶는다. 설정은 `src/cortex_bringup/config/logging.yaml`.

## 어디에 남나

```
~/cortex_logs/
  sessions.csv                                  세션 한 줄씩: 시각, 발화, 결과, 소요 시간, 단계 수, 파일
  2026-10-15/
    160404_p-1792015432-0001.jsonl              이벤트 한 줄씩 (JSON)
    160413_p-1792015432-0002.jsonl
```

- Docker: `docker/run.sh` 가 호스트의 `CORTEX_LOG_DIR` (기본 `~/cortex_logs`) 를 컨테이너의
  `/root/cortex_logs` 에 마운트한다. 컨테이너를 지워도 남는다. 파일 소유자는 root 다.
  이 마운트가 없는 예전 컨테이너는 재사용하지 않고 멈춘다 (다시 만들라는 안내가 나온다).
- 시각은 호스트 시간대 기준이다 (`/etc/localtime` 공유).
- 14일(`keep_days`)보다 오래된 날짜 폴더는 recorder 가 지운다. `sessions.csv` 는 지우지 않는다.

## 세션 하나에 들어가는 것

- 시작 직전 5초(`pre_roll_s`)의 이벤트: 전사, STT 세션 상태
- `SESSION_START` ~ `SESSION_END` 사이의 이벤트. 그 세션 `plan_id` 가 붙은 이벤트는 그 세션에만,
  `plan_id` 가 없는 이벤트(stt·tts·detector)는 열려 있는 모든 세션에 들어간다.
- 끝난 뒤 2초(`post_roll_s`): 마지막 TTS 문장 등

무엇을 기록할지는 `logging.yaml` 의 `record` 로 정한다 (기본 `all`). 콘솔 출력(`events`)과는 따로다.

## 결과(outcome)

| 값 | 뜻 |
|---|---|
| `succeeded` | 마지막 단계까지 완료 |
| `failed` | 단계 실패·수락 안 됨·모듈 정지 실패 (detail 에 사유) |
| `stopped` / `preempted` | 사용자 정지 / 새 명령에 밀림 |
| `reply` / `confirm` | 대답만 함 / 되물음 |
| `plan_error` / `plan_timeout` | 계획 차단·오류 / 계획 응답 없음 |
| `replaced` / `dropped` | 계획 전에 다시 말함 / 대기 발화가 실패 정지와 함께 버려짐 |
| `recorder_timeout` / `recorder_stopped` | 120초 안에 END 없음 / 노드 종료로 닫힘 |

## 보기

```bash
column -s, -t < ~/cortex_logs/sessions.csv | tail
```

```bash
jq -c '[.node, .event, .fields]' ~/cortex_logs/2026-10-15/160404_p-1792015432-0001.jsonl
```

끄려면 `logging.yaml` 에서 `session_log.enabled: false`.
