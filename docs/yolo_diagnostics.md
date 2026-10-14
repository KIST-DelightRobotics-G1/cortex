# 냉장고 미검출 현장 진단 — RTX 4090 / Ubuntu / ROS 2 Humble

대상 브랜치: `SYS-REQ-44-yolo-diagnostics`. 기존 모델·confidence·연속 프레임
판정·CheckTarget 메시지·Subtask 실행 조건을 유지한다. 추가 토픽은 진단용이며
로봇 제어에 사용하지 않는다. 표준 launch는 회차별 텍스트 로그와 진단 bag을 자동 저장한다.

## 1. 준비

현장 Cortex와 같은 Ubuntu 22.04 / ROS 2 Humble, Python 3.10 환경을 사용한다.
이 브랜치를 체크아웃한 저장소 루트에서 실행한다. 기존 작업이 있다면 별도
작업 폴더를 사용한다. 모델은 기존 파일을 그대로 사용하고 Git에 포함하지 않는다.

```bash
source /opt/ros/humble/setup.bash
# 기존에 사용하던 ROS_DOMAIN_ID, RMW_IMPLEMENTATION, DDS 설정을 동일하게 적용
colcon build --symlink-install
source install/setup.bash
nvidia-smi
python3 -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CUDA UNAVAILABLE")'
```

CUDA가 없으면 GPU 검증을 진행하지 않는다. 필요한 경우 기존 저장소의
`requirements-torch-cu126.txt`, `requirements.txt`, `requirements-detector.txt`에
맞춰 설치한다. CUDA PyTorch는 드라이버가 보이는 호스트 또는 GPU가 노출된
컨테이너에서 실행해야 한다. Docker에서는 **새 브랜치로 이미지를 재빌드**하거나
해당 컨테이너 안에서 브랜치 소스를 빌드해야 한다. 이전 이미지에 호스트
브랜치만 변경해도 반영되지는 않는다. NVIDIA Container Toolkit과 `--gpus`가
필요하며 컨테이너 안에서도 위 두 GPU 확인 명령을 실행한다.

## 2. Cortex 실행과 회차별 자동 저장

이미 실행 중인 Cortex를 종료한 뒤 같은 ROS 환경에서 실행한다.
동일 이름의 detector/orchestrator를 동시에 두 개 실행하지 않는다.

```bash
ros2 launch cortex_bringup cortex.launch.py \
  model:=/models/cortex/yolo26s_fridge_cucumber_continued.pt device:=0
```

기본값인 `record_session:=true`에서는 **launch 한 번을 실험 한 회차**로 기록한다.
별도 수집 터미널이 필요하지 않다. detector/orchestrator의 진단 출력만 자동으로
켜며, 모델·confidence·window·다수결 등 판정 조건은 바꾸지 않는다.
현장 전체 설정은 `params_file:=/absolute/path/site.yaml`, 기존 프로파일은
`detector_profile:=/absolute/path/profile.yaml`로 그대로 지정할 수 있다.

사전조건 제한시간과 검출 기준은 현장 YAML의 기존 값을 따른다. 자동 기록은
`precheck_timeout_s`, `precheck_retry_s`, `window_s`를 변경하지 않는다.
실제 적용값은 회차 폴더의 파라미터 덤프에서 확인한다.
모델 해시는 **실제 파일과 현장 설정이 일치해야 한다**. 표준 파일은
`3d275b84378da06dc9202b4d40962f49f8d88d18062357d1d167abab10d6f920`이다.
카메라와 nav/VLA 등 외부 모듈은 평소와 같이 별도로 실행한다.
검출만 확인하려면 `detector_only:=true`를 추가한다.

시작 시 `Cortex session logs: ...`와 ROS launch의 로그 경로가 표시된다.
기본 위치는 `~/.ros/log/<날짜-호스트-PID>/`이며, 실행 전에 `ROS_LOG_DIR`을
지정하면 그 아래에 회차 폴더가 생성된다. 결과 구성은 다음과 같다.

- `launch.log`: 이 launch가 시작한 전체 Cortex 노드의 stdout/stderr와 시작·종료 로그.
- 노드별 ROS 로그: 같은 회차 폴더에 저장.
- `diagnostics/bag/`: 카메라, 검출, YOLO/precheck 진단, STT 인식 문장,
  LLM 요청·단계, TTS 요청·중단, nav/VLA 명령·상태, task_status, trace, rosout.
- `diagnostics/detector_params.yaml`, `orchestrator_params.yaml`: 실행 중인 판정 설정.
- `diagnostics/manifest.json`, `record.log`: 기록 상태, 토픽별 수집 수, 누락 토픽,
  환경 확인 결과. 접근 가능한 모델 파일은 SHA-256도 기록한다.

텍스트 로그는 노드 시작부터 남는다. bag 수집은 노드 파라미터 확인 후 시작하므로
**시작 직후의 모든 토픽을 소급 저장하지는 않는다**. 노드 준비는 최대 45초간
재시도하며, 준비 실패 시 manifest와 launch 로그에 수집 실패가 남는다.
`diagnostics/record.log`에서 필요한 토픽의 구독이 시작됐는지 확인한 후 시험한다.
기록 중 시험 조건과 성공/실패 시각을 메모한다. 정지 후 확인, 도착 직후 확인,
화각 변화, 짧은 가림을 비교하고, 부재·영상 중단 시험은 detector-only에서 진행한다.

종료는 기존처럼 **Ctrl+C 한 번 후 기록 종료까지 기다린다**. 수집기는 자신이
시작한 rosbag만 종료하며 메타데이터가 저장될 시간을 준다. 수집 실패가 로봇
명령이나 판정 결과를 바꾸지는 않는다. 강제 종료·디스크 부족·수집 실패 시
부분 기록일 수 있으므로 `manifest.json`의 status, recorded_topic_counts,
missing_topics를 확인한다. `recorded`도 모든 토픽의 무손실 저장을 의미하지 않는다.
카메라가 0건이면 `incomplete_no_camera`로 표시한다. detector-only에서는
orchestrator 관련 토픽이 없는 것이 정상이다. 외부 nav/VLA 프로세스의
파일 로그까지 복사하는 기능은 아니며, 발행된 상태 토픽과 rosout을 함께 수집한다.

Docker는 새 이미지와 로그 마운트가 적용된 컨테이너에서 실행한다.

```bash
CORTEX_LOG_DIR="$HOME/cortex-logs" docker/run.sh \
  ros2 launch cortex_bringup cortex.launch.py
```

`CORTEX_LOG_DIR` 기본값은 호스트의 `~/.local/state/cortex/logs`이다.
컨테이너 내부의 `/logs/cortex`에 연결되며, **호스트의 해당 회차 폴더 전체**를
압축해 전달하면 된다. 기존 컨테이너의 마운트가 다르면 스크립트가 재사용을
거부한다. 필요한 파일을 먼저 보존한 뒤 컨테이너를 명시적으로 재생성한다.
스크립트가 기존 컨테이너를 자동 삭제하지는 않는다.

영상 수집 부하를 제외하고 텍스트 로그만 저장하려면 `record_session:=false`를
사용한다. 이 경우 진단 활성화 여부는 YAML을 따른다. 수동으로 90초만 수집하려면
`record_session:=false`와 `yolo_diagnostics.yaml` 프로파일로 실행한 뒤 아래
기존 도구를 사용한다. 자동 수집과 동시에 실행하지 않는다.

```bash
python3 scripts/collect_yolo_diagnostics.py --duration 90 --output results/fridge-run01
```

수동 수집도 ROS와 install/setup.bash를 source한 동일 환경에서 실행한다.
자동 수집과 수동 수집 모두 명령 발행·서비스 판정 요청·파라미터 변경을 하지 않는다.
rosbag은 영상 크기와 시험 시간에 따라 저장 공간과 부하를 사용한다. 단독/통합
비교에서는 같은 기록 조건을 유지한다. 결과에는 발화 텍스트와 영상이 포함되므로
공유 전 확인하며, .env·인증 파일·마이크 음성은 별도로 복사하지 않는다.

## 3. 로그와 영상을 읽기 쉬운 파일로 추출

ROS를 source한 동일 환경에서 실행한다. 기본 H.264뿐 아니라 raw/compressed
영상도 지원한다. 현장 카메라 토픽이 다르면 --camera-topic을 지정한다.
아래는 수동 수집 경로 예시이며, 자동 저장분은 --bag에
<회차 폴더>/diagnostics/bag 경로를 지정한다.

```bash
python3 scripts/export_yolo_bag.py --bag results/fridge-run01/bag \
  --output results/fridge-run01-export
```

결과는 `camera.mp4`, `video_frames.jsonl`, `events.jsonl`, `export_summary.json`이다.
원래 프레임 시각을 보존하지만 MP4는 재인코딩된 영상이다. 픽셀 수준의
정밀 비교에는 원본 영상과 bag도 함께 전달한다. H.264는 시작 키프레임까지
영상이 없을 수 있으며, 한 패킷에 한 디코딩 프레임이 반드시 나오지는 않는다.

## 4. 독립 YOLO 분석 — ROS 불필요

모델과 MP4가 있으면 아래 명령 한 번으로 실행한다. 배포 모델과 동일한
라이브러리를 사용한다. 기존 저장소 또는 전달된 독립 도구 묶음 루트에서 실행한다.
별도 Python 환경이라면 CUDA용 requirements-torch-cu126.txt를 먼저 설치하고
requirements-yolo-diagnostics.txt를 설치한다. 기존 Cortex 환경에서는 중복 설치가 필요 없다.

```bash
python3 scripts/diagnose_yolo.py \
  --model /models/cortex/yolo26s_fridge_cucumber_continued.pt \
  --video results/fridge-run01-export/camera.mp4 \
  --device 0 --output results/fridge-run01-offline \
  --profile results/fridge-run01/detector_params.yaml
```

실행 중 detector의 파라미터 덤프를 --profile로 쓰면 현장 설정으로 분석한다.
덤프가 없으면 --profile을 생략해 저장소 기본 설정을 쓸 수 있다.
--video를 여러 번 지정하면 여러 영상을 순서대로 분석한다. 현장 CUDA가 없으면
자동 CPU 전환 없이 종료한다. 다른 의도된 모델을 시험하려면
--expected-sha256에 그 파일의 해시를 명시한다.

| 결과 | 해석 |
|---|---|
| manifest.json | 모델·코드 해시, GPU, 라이브러리, 설정, 전체 완료 여부 |
| 각 영상/annotated.mp4 | 현장 infer_conf에서 검출한 박스와 confidence. 고정 FPS 미리보기 |
| 각 영상/frames.jsonl | 모든 프레임의 원본 시각, 박스, 낮은 confidence 후보, baseline 처리시간 |
| 각 영상/policy.csv | 8Hz 등 현장 주기에 맞춘 판정 표본과 차단 이유 |
| 각 영상/timeline.svg | 기준별 냉장고·오이 허용/차단/확인불가 시점 |
| 각 영상/summary.json | 차단 사유별 횟수, 처리시간 평균·p50·p95, 초기 워밍업 시간 |

0.15/0.20/0.25 비교는 탐색 분석이다. 낮은 confidence 후보를 별도로 추론하며
기존 기준의 박스·처리시간에 섞지 않는다. 현장 설정은 변경하지 않는다.
`--source-delay-ms 400`은 가상 지연을 넣는 추가 실험이며 실측 지연이 아니다.

저장 영상에서는 실제 DDS·CPU 경합·패킷 누락·시계 오차·실행기의 사전조건 재시도를
재현하지 않는다. 판정은 추론 완료 시점 표본이며 Subtask 시작 성공률이 아니다.
정답 라벨이 없으므로 결과의 allow 비율을 Precision/Recall이라고 해석하지 않는다.
기존 test 영상으로 설정을 비교할 때는 탐색 분석이며 독립 일반화 검증이 아니다.
워밍업·디코딩·두 번째 confidence 분석을 제외한 baseline 추론 시간을 보고한다.
전체 로봇 지연이나 GPU 처리량과 같지 않다. MP4 미리보기와 달리 CSV/JSONL의
프레임 시각이 분석 기준이다. VFR 영상에서 시간정보가 없으면 명시적으로
FPS 대체 여부를 기록하며, 모순된 시각은 오류로 종료한다.

## 5. 추가 로그 해석

진단 토픽은 `std_msgs/String` JSON이며 `/cortex/detector/diagnostics`에
1초 주기의 heartbeat와 실제 CheckTarget 호출마다 check 이벤트가 나온다.
`/cortex/precheck/diagnostics`에는 plan_id/index/target, 서비스 응답시간과
응답 결과가 나온다. 서비스 요청 ID는 기존 규약에 없으므로 양쪽 이벤트는
시각·target으로 비교한다. 여러 클라이언트가 동시에 요청하면 일대일 상관은
보장하지 않는다. task_status/trace의 최종 차단 이유도 함께 확인한다.

| 진단 reason/카운터 | 의미 |
|---|---|
| no_hits | 판단 구간에서 기준 이상 검출 없음 |
| below_majority | 검출은 있으나 과반수 미달 |
| latest_miss | 과반수 검출됐으나 마지막 프레임에서 놓침 |
| insufficient_frames | 최근 유효 프레임 3개 미만. hits가 있어도 시작하지 않음 |
| no_frame / stale | 처음부터 유효 프레임 없음 / 기존 영상이 만료됨 |
| stale_input / future_stamp / unstamped | 영상 시각이 너무 과거·미래이거나 없음 |
| stale_before_inference / window_add_rejected | 처리 전 또는 완료 시점에 유효기간 등 조건 미충족 |
| h264_waiting_keyframe / h264_decode_error | 키프레임 대기 / 디코더 오류. 패킷 손실 자체를 확정하는 지표는 아님 |
| h264_buffering | 디코더가 아직 프레임을 출력하지 않음. 오류와 구분 |
| detector_timeout | orchestrator의 서비스 응답 대기 초과 |

`max_candidate_confidence`는 **추론의 infer_conf를 통과한 후보만**의 점수다.
낮은 후보는 독립 분석에서 확인한다. CheckTarget 부정 응답의 confidence=0은
원래 YOLO 점수가 0이었다는 뜻이 아니다. 진단 hits는 프레임 부족 단계에서도
집계하므로 기존 서비스의 내부 Verdict.hits와 다를 수 있다.
진단은 best-effort이며 수집 손실은 가능하다. 이를 제어용 근거로 사용하지 않는다.

## 6. 회신 자료

자동 저장한 회차 폴더 전체(launch.log와 diagnostics 포함), 추출 결과 폴더,
독립 분석 결과 폴더와 시험 조건·성공/실패
시각 메모를 함께 전달한다. 모델 파일은 기존 파일을 유지하고 해시로 확인한다.
수집기는 .env·인증키·마이크 음성을 복사하지 않지만 trace/rosout에는 발화문이
들어갈 수 있으므로 공유 전 확인한다. 결과 폴더에 ERROR.txt가 있거나
manifest가 complete/recorded가 아니면 부분 실행임을 함께 알려준다.
