# 운용 점검 가이드 (Operations checklist)

cortex를 띄우고 **STT → 인지 → TTS → 스피커** 순서대로 한 단계씩 확인하는 절차.
각 단계는 "실행할 명령 / 정상이면 보이는 것 / 아니면 어디가 문제인지"로 되어 있다.
위에서부터 순서대로 하고, 실패한 단계에서 멈춘 뒤 §5 증상표를 본다.

> 이 문서는 명령어와 기댓값만 다룬다. 설계 배경은 [README](../README.md),
> detector는 [detector_integration.md](detector_integration.md)를 본다.

---

## 0. 터미널 준비

점검에는 터미널 **2개**가 필요하다. 둘 다 아래를 먼저 실행한다.

```bash
cd ~/cortex            # 저장소 경로에 맞게
source env.sh
```

정상이면 이렇게 보인다:

```
[env.sh] loaded .env
[env.sh] Activated ROS humble with rmw_cyclonedds_cpp
[env.sh]   ROS_DOMAIN_ID=0  DDS_PEER_IP=192.168.123.164  DDS_ROBOT_IP=192.168.123.161  CORTEX_NIC=eno2
```

| 보이는 것 | 뜻 / 할 일 |
|---|---|
| `WARN: no .env` | 자격증명 파일이 없다. `cp .env.example .env` 후 키를 채운다. 이 상태로 띄우면 LLM은 dummy, STT/TTS는 동작하지 않는다 |
| `CORTEX_NIC=eno2`인데 로봇 미연결 | `eno2`가 DOWN이면 DDS가 아예 못 뜬다. `export CORTEX_NIC=lo` 후 다시 `source env.sh` |
| `ROS_DOMAIN_ID`가 0이 아님 | 다른 모듈과 못 만난다. 0이어야 한다 |

빌드가 안 돼 있으면 한 번만:

```bash
colcon build --symlink-install
```

> **⚠️ `cortex_params.yaml`을 수정했다면 반드시 다시 빌드한다.** 설정 파일은
> `install/`로 복사되므로, 빌드하지 않으면 **수정 전 설정으로 실행된다.** 이게
> 가장 흔한 함정이다 (§1-4에서 확인).

---

## 1. 사전 점검 (Preconditions)

### 1-1. ext-sensor-io가 센서를 내보내고 있는가

cortex는 마이크·카메라를 직접 열지 않고 `ext-sensor-io`가 발행하는 토픽을 받는다.
**cortex보다 먼저 ext-sensor-io가 떠 있어야 한다.**

```bash
ros2 topic hz /kist/mic/uno/audio
```

| 결과 | 판정 |
|---|---|
| `average rate: 10.0` 근처 | ✅ 정상 (100 ms 청크 = 초당 10개) |
| `no new messages` 또는 멈춤 | ❌ ext-sensor-io 미실행 또는 마이크 미연결. **cortex 문제가 아니다** |

카메라도 같이 본다 (detector·화면용):

```bash
ros2 topic hz /kist/camera/head/color/h264
```

**마이크가 어느 장치인지 확인** — 설정에 따라 둘 중 하나다:

| 토픽 | 장치 | 포맷 |
|---|---|---|
| `/kist/mic/array/audio` | reSpeaker Flex XVF3800 | 16 kHz, 6채널. **빔포밍·에코 감소 적용됨** |
| `/kist/mic/uno/audio` | ESI NEVA UNO | 44.1 kHz, 2채널. 빔포밍·에코 감소 **없음** |

cortex 기본값은 데모 PC의 `uno`다. 하드웨어 에코 억제가 없으므로 §4 에코 점검이
중요하다. 로봇의 `array`를 쓰려면 `stt_node.audio_topic`을 바꾼다.

### 1-2. cortex 기동

터미널 1:

```bash
./scripts/run_cortex.sh
```

### 1-3. 노드 7개가 모두 떴는가

터미널 2:

```bash
ros2 node list
```

**7개가 모두 나와야 한다:**

```
/detector_node
/gui_bridge_node
/llm_node
/orchestrator_node
/speaker_node
/stt_node
/tts_node
```

하나라도 빠지면 터미널 1의 로그에서 그 노드의 에러를 찾는다. 기동 로그에서 특히
아래 네 줄을 확인한다:

```
[stt_node]  stt_node up (backend=google_cloud, model=latest_short, end_timeout=0.5s [v2 only], lang=ko-KR, ...)
[tts_node]  tts_node up (backend=naver_clova, voice=nara, say=/cortex/tts/say -> /cortex/tts/audio)
[llm_node]  llm_node up (backend=gemini, model=gemini-3.6-flash, 20 actions, 5 places)
[orchestrator_node] orchestrator_node up (mode=llm, tick=10.0Hz)
```

| 로그 | 뜻 |
|---|---|
| `llm_node: backend=gemini but no API key ... falling back to dummy` | `.env`에 `GOOGLE_API_KEY`가 없다. LLM이 실제로 동작하지 않는다 |
| `stt_node ... backend=dummy` | 자격증명 없이 뜬 것. 실제 음성 인식 안 됨 |
| `tts_node ... NCP_CLOVA_... not set` | CLOVA 키 없음. 말을 못 한다 |
| `model=default` (stt) | ⚠️ **설정이 반영되지 않았다.** §1-4로 |

### 1-4. 설정이 실제로 반영됐는가

가장 자주 터지는 문제다. YAML을 고쳤는데 재빌드하지 않으면 옛 설정으로 돈다.

```bash
ros2 param get /stt_node model          # → latest_short  (default 가 나오면 재빌드 필요)
ros2 param get /stt_node backend        # → google_cloud
ros2 param get /stt_node audio_topic    # → /kist/mic/uno/audio
ros2 param get /llm_node backend        # → gemini (dummy 면 GOOGLE_API_KEY 가 없는 것)
ros2 param get /orchestrator_node planner_mode   # → llm
```

`model`이 `default`로 나오면 **최종 인식 결과가 매우 늦게 나온다**(스트림이 닫힐 때까지
기다림). `latest_short`여야 말이 끝나고 바로 결과가 나온다.

### 1-5. 토픽과 서비스가 모두 떴는가

```bash
ros2 topic list | grep cortex
```

아래가 모두 있어야 한다:

```
/cortex/detections          /cortex/llm/request      /cortex/tts/audio
/cortex/llm/step            /cortex/speaker/state    /cortex/tts/prefetch
/cortex/nav/cmd             /cortex/stt/transcript   /cortex/tts/say
/cortex/nav/state           /cortex/task_status      /cortex/tts/stop
/cortex/trace               /cortex/vla/cmd          /cortex/tts/warmup
                            /cortex/vla/state
```

detector 서비스:

```bash
ros2 service list | grep detector       # → /cortex/detector/check
```

---

## 2. STT 확인 (음성 → 글자)

터미널 2에서 전사 결과를 띄워놓는다:

```bash
ros2 topic echo /cortex/stt/transcript
```

마이크에 **"냉장고에서 오이 가져와"** 라고 또렷하게 말한다.

정상이면 말이 끝난 **1초 안에** 아래가 나온다:

```yaml
data: 냉장고에서 오이 가져와
---
```

터미널 1에도 같이 찍힌다:

```
[stt_node] transcript: '냉장고에서 오이 가져와' (final=True, conf=0.87)
```

| 증상 | 확인할 것 |
|---|---|
| 아무것도 안 나옴 | §1-1 마이크 토픽부터 다시. 그 다음 §4 에코 뮤트 |
| 10초 이상 뒤에 나옴 | `ros2 param get /stt_node model` → `latest_short`인지 (§1-4) |
| 엉뚱한 글자 | 마이크 장치(`array` vs `uno`)와 주변 소음 확인 |
| 로그에 `Google stream error` 반복 | 자격증명 또는 네트워크 문제 |

> `conf` 값은 신뢰하지 말 것. 서로 다른 발화에 같은 값이 찍히는 것이 관측되었다.

---

## 3. 인지 → 발화 확인

### 3-1. 마이크 없이 명령 주입 (가장 유용한 테스트)

마이크·음성 없이 뒷단만 시험할 수 있다. 전사 토픽에 직접 글자를 넣는다:

```bash
ros2 topic pub --once /cortex/stt/transcript std_msgs/msg/String "{data: '냉장고에서 오이 가져와'}"
```

터미널 1에서 이 순서로 보여야 한다:

```
[orchestrator_node] ... (요청 접수)
[tts_node] say '네.': cache hit, first publish +3 ms          ← 접수 응답(ack)
[llm_node] plan p-...-0001: '냉장고에서 오이 가져와' (mode=idle, ...)
```

### 3-2. 계획이 나오는가

터미널 2에 띄워놓고 위 명령을 다시 보낸다:

```bash
ros2 topic echo /cortex/llm/step
```

`kind: 0`(서브태스크)이 여러 개 오고 마지막에 `kind: 1`(END)이 와야 한다.
`kind: 3`은 오류이고 `detail`에 이유가 들어 있다.

| `kind` | 뜻 |
|---|---|
| `0` SUB | 서브태스크 — `action`, `args`, `say` |
| `1` END | 계획 완료 |
| `2` REPLY | 계획 없이 대답만 (잡담·거부) |
| `3` ERROR | 실패 — `detail` 확인 |

> `llm_node`가 `backend=dummy`면 실제 LLM이 아니라 예시 문장을 재생한다. 기본은
> `gemini`이며 `.env`에 `GOOGLE_API_KEY`가 없으면 dummy로 떨어진다. 키 없이 일부러
> 오프라인으로 보려면: `ros2 launch cortex_bringup cortex.launch.py llm_backend:=dummy`

### 3-3. TTS 단독 확인

인지 단계를 건너뛰고 말하기만 시험한다:

```bash
ros2 topic pub --once /cortex/tts/say cortex_msgs/msg/ActionCmd "{text: '테스트입니다'}"
```

터미널 1:

```
[tts_node] published 20480 bytes in 1 chunk(s)
[tts_node] say '테스트입니다': cache miss, first publish +498 ms
```

| 로그 | 뜻 |
|---|---|
| `cache hit, +3 ms` | 전에 합성해 둔 문장 (정상, 가장 빠름) |
| `cache miss, +400~1400 ms` | CLOVA에 새로 요청함 (정상) |
| `no credentials — dropping` | CLOVA 키 없음 |
| 아무 로그 없음 | `say_topic` 이름 불일치 |

**중요: 이 로그가 찍혀도 "소리가 났다"는 뜻은 아니다.** 합성해서 발행만 한 것이다.
실제 재생은 §3-4에서 확인한다.

### 3-4. 실제로 소리가 나는가 ★

합성은 성공했는데 아무도 재생하지 않아 조용한 경우가 실제로 있었다. 반드시 확인한다.

```bash
ros2 topic info /cortex/tts/audio
```

```
Publisher count: 1        ← tts_node
Subscriber count: 1       ← speaker_node. 0 이면 소리가 나지 않는다
```

`Subscriber count: 0`이면 `speaker_node`가 없거나 다른 토픽을 보고 있다:

```bash
ros2 param get /tts_node audio_out_topic       # → /cortex/tts/audio
ros2 param get /speaker_node audio_topic       # → /cortex/tts/audio  (둘이 같아야 한다)
```

두 값이 다르면 소리가 안 난다. 로봇 스피커로 나가는 경로는
`tts_node → /cortex/tts/audio → speaker_node → 로봇 오디오 서비스`이고,
마지막 구간은 `DDS_ROBOT_IP`(기본 `192.168.123.161`)를 쓴다.

---

## 4. 에코 차단 확인 (로봇이 자기 말을 듣지 않는가)

로봇이 말할 때 마이크를 막지 않으면 **자기 말을 명령으로 다시 인식**해 무한 반복에
빠질 수 있다. 차단은 `speaker_node`가 발행하는 상태 신호로 이뤄진다.

```bash
ros2 topic echo /cortex/speaker/state
```

`ros2 topic pub --once /cortex/tts/say ...`(§3-3)을 보내고 관찰한다:

```yaml
playing: true        ← 재생 중 (이 동안 stt_node 가 마이크를 버린다)
...
playing: false       ← 재생 끝 (200 ms 더 버린 뒤 복귀)
```

| 증상 | 뜻 |
|---|---|
| 토픽에 아무것도 안 옴 | `speaker_node` 미실행 → **에코 차단이 전혀 동작하지 않음** |
| `playing: true`에서 안 바뀜 | 재생이 멈췄는데 신호가 남은 상태. **마이크가 계속 막혀 STT가 먹통이 된다.** `speaker_node` 재시작 |

두 토픽 이름이 맞는지:

```bash
ros2 param get /stt_node speaker_state_topic    # → /cortex/speaker/state
ros2 param get /speaker_node state_topic        # → /cortex/speaker/state
```

### 끼어들기(barge-in)

말하는 중에 멈추는 기능:

```bash
ros2 topic pub --once /cortex/tts/stop std_msgs/msg/Bool "{data: true}"
```

음성으로는 **"그만" / "멈춰" / "정지" / "스톱"** 중 하나를 말한다.

---

## 5. detector 확인 (물체가 보이는가)

인지 단계는 동작을 내보내기 전에 "대상이 실제로 보이는지" detector에 묻는다.
**카메라가 안 떠 있으면 동작이 차단된다**(`detector_fail_open: false`).

```bash
ros2 service call /cortex/detector/check cortex_msgs/srv/CheckTarget \
  "{target: 'cucumber', min_confidence: 0.0, max_age_s: 0.0}"
```

```yaml
found: true
confidence: 0.91
label: cucumber
detail: ''              ← '' 가 정상 판정
```

`detail` 값으로 원인을 안다:

| `detail` | 뜻 |
|---|---|
| `''` | 정상 판정 |
| `stub` | 데모용 가짜 백엔드 (`backend: always`) |
| `no_frame` | 카메라 프레임이 안 들어옴 → §1-1 카메라 확인 |
| `stale` | 프레임이 너무 오래됐음 |
| `model_not_loaded` | 가중치 파일 경로 문제 |
| `unknown_target` | 그 이름을 모름 (`actions.yaml` 어휘 확인) |

---

## 6. 화면(GUI) 확인

```bash
ros2 topic hz /cortex/trace       # 생각의 흐름 이벤트
```

`gui_bridge_node`는 기동 로그에 `ws://0.0.0.0:8081`을 찍는다. 화면 렌더러는
별도 컨테이너(cortex-gui)다.

---

## 7. 정상 종료

터미널 1에서 **Ctrl-C** 한 번. 노드들이 차례로 내려간다.

> 종료할 때 모든 노드가 `rcl_shutdown already called` 와 함께 `exit code 1`로
> 죽는다면, **그 체크아웃이 오래된 것이다.** 최신 `main`에는 수정되어 있다.
> `git pull` 후 `colcon build --symlink-install`.

---

## 증상별 빠른 진단표

| 증상 | 가장 먼저 볼 곳 |
|---|---|
| 말했는데 아무 반응 없음 | §1-1 마이크 토픽 → §2 전사 → §4 `playing: true` 고착 |
| 전사가 10초 이상 늦음 | §1-4 `model` = `latest_short` 인지 |
| 로봇이 대답을 안 함(글자는 나옴) | §3-3 TTS 로그 → §3-4 `Subscriber count` |
| 로봇이 말하는데 소리가 안 들림 | §3-4 ★ (합성 성공 ≠ 소리 남) |
| 로봇이 자기 말에 반응해 반복됨 | §4 `/cortex/speaker/state` 가 오는지 |
| 로봇이 주변 대화에 반응함 | 현재 웨이크워드가 없다 — 알려진 제약 |
| 동작이 시작되지 않음 | §5 detector `detail` 확인 |
| LLM이 이상한 답을 함 | §1-3 `backend=dummy` 인지 (dummy는 예시 재생) |
| 설정을 바꿨는데 그대로임 | §1-4 → `colcon build --symlink-install` |
| 종료할 때 전부 exit 1 | §7 — 오래된 체크아웃 |

---

## 한 번에 복사해서 쓰는 사전 점검

```bash
source env.sh
ros2 topic hz /kist/mic/uno/audio            # Ctrl-C 로 중단. 10 Hz 근처여야 함
ros2 node list                               # 7개
ros2 param get /stt_node model                # latest_short
ros2 param get /stt_node backend              # google_cloud
ros2 param get /llm_node backend              # gemini
ros2 param get /orchestrator_node planner_mode  # llm
ros2 topic info /cortex/tts/audio             # Subscriber count: 1
ros2 service list | grep detector             # /cortex/detector/check
```
