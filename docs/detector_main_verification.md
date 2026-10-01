# 최신 main과 YOLO precondition 통합 확인

`SYS-REQ-44-yolo-detector-integration` 로컬 브랜치에 원격 main의
`2dee951`을 병합했다. 기존 YOLO 구현 커밋 `9c1ed90`을 보존했으며,
main의 카메라·실행기 변경에 맞춰 통합부와 검증 도구를 수정했다.
원격 push와 실제 로봇 구동은 수행하지 않았다.

## 적용한 변경

- **영상 입력:** 기본 입력을 `/kist/camera/head/color/h264`의
  `kist_msgs/CompressedColorFrame`으로 연결했다. H.264는 순차 디코딩하며,
  추론은 최신 영상에서 목표 8 Hz로 수행한다. 디코더가 과거 프레임을 늦게
  반환해도 해당 프레임의 촬영 시각을 유지한다. 시각 없는 프레임은 판정에
  넣지 않으며, 기존 raw·compressed 입력도 유지한다.
- **실행 흐름:** 객체 확인 → 명령 발행 → DONE → IDLE → 다음 객체 확인
  순서로 처리한다. DONE만으로 다음 동작을 보내지 않는다.
- **정지 흐름:** 명령 발행 전 PRECHECK 중 정지는 모듈 취소 없이 대기를
  종료한다. 발행 후에는 main의 cancel·IDLE 대기 규칙을 유지한다.
- **계획 보정:** main의 `approach`·`step_back` 삽입 규칙을 보존했다.
  `open → approach → pick → step_back → close`로 실행되며, 원래 단계의
  냉장고·오이·냉장고 확인은 각각 실행 직전에 수행한다. 삽입 단계에는
  main과 동일하게 별도의 객체 확인을 추가하지 않았다.
- **전체 노드 실행:** 전용 YOLO launch에 main의 `speaker_node`를 포함했다.

선택 가중치와 confidence 0.25·0.6초 과반·최신 프레임 확인·최대 1초 재확인
설정은 유지했다. 모델 클래스는 냉장고와 오이이며 문 상태·도달 가능성·완료
여부는 이 검출기로 판단하지 않는다. 실행 및 설치 절차는 [v3 설정](detector_v3.md)을 참고한다.

## 검증 결과

| 검사 | 결과 |
| --- | --- |
| 로컬 자동 테스트 | 105 passed, 1 skipped |
| DONE 후 IDLE 대기·재확인·정지 | 통과; 다음 명령 및 취소 발생 여부 확인 |
| H.264 디코더 | B-frame 지연 시 원본 PTS 보존, 손상 후 키프레임 복구, 시각 없음 처리 통과 |
| 실제 가중치 + H.264 영상 | KIST 근접 영상 일부 16프레임 처리, 16개 촬영 시각 보존 |
| 입력 만료 | 오래된 프레임·검출 결과로 실행 허용되지 않음 |

실제 가중치 확인은 근접 영상의 약 12초·26초 구간에서 각각 8프레임을 뽑아
Annex-B H.264로 다시 인코딩한 후 공통 디코더·YOLO 어댑터·존재 판정기로
처리했다. 각 구간 초기 2프레임의 누적 대기 이후, 오이 가시 구간 6회는
냉장고·오이 모두 허용했고 부재 구간 6회는 냉장고만 허용했다.
**연결 검증을 위한 작은 표본이며 정확도 평가나 추가 학습 결과가 아니다.**
시간은 오프라인 모의 시계이며 30 ms 입력 나이를 가정했다. 실측 지연시간이 아니다.

## 녹화 영상 기반 Subtask 재생

기존 실제 모델 추론 결과와 현재 Cortex 실행기를 연결했다. main의 시연용
계획 보정을 켰으며, VLA heartbeat·DONE·IDLE은 모의 입력했다.

| 시나리오 | 관측 결과 |
| --- | --- |
| 오이가 잠시 뒤 보임 | open 0.4초 → approach 1.3초 → pick 11.1초 → step_back 23.3초 → close 23.8초, 마지막 IDLE 후 소프트웨어 계획 완료 |
| 오이가 없는 시점에 pick 요청 | open·approach 이후 재확인 시간 초과; pick 및 이후 명령 없음 |
| DONE 없음 | open 이후 다음 단계 없음 |
| DONE 있으나 IDLE 없음 | open 이후 다음 단계 없음; IDLE 대기 시간 초과로 실패 |
| 최초 객체 확인 대기 중 정지 | VLA 명령 0건, 대기 종료 |

첫 시나리오의 모의 DONE은 1.0 / 10.3 / 23.0 / 23.5 / 25.2초이고,
각각 0.3초 뒤 IDLE을 입력했다. pick 확인은 10.6초에 시작해 11.1초에
허용했다. 이 시각들은 수동으로 정한 **녹화 영상 재생 시각**으로 실제 작업
시간이나 로봇 동작 성공을 뜻하지 않는다. 오이 존재 판정 비교 수치는
[병합 전 검증 기록](detector_v3_verification.md)과 동일하다.

## 검증 범위

macOS의 기존 실험용 Python 3.12 환경에서 수행했다. ROS가 없어 생성 메시지와
실제 노드 검사를 포함한 테스트 모듈 1개가 제외됐다. Docker/colcon도 설치되어
있지 않아 Humble 빌드·컨테이너 검사·DDS·실제 카메라·VLA 연결은 수행하지
않았다. 기존 CI 설정은 유지했으며 원격 CI는 실행하지 않았다.

실제 입력에서 확인할 항목은 촬영 시각과 수신 측 시각 동기화, H.264 전송/QoS,
목표 PC의 처리시간, VLA가 DONE 뒤 IDLE을 보내는지 여부다. 녹화 test를 재사용한
탐색 확인이므로 KIST에서 precondition을 적용한 별도 실행 기록으로 확인해야 한다.

재생기는 `scripts/replay_preconditions.py`이며, 로컬 원시 검증 자료는
`/Volumes/T7/g1_experiments/cortex_main_integration_20261001/`에 보관했다
(`precondition_replay.json`, `h264_smoke.json`, `h264_smoke.py`, `pytest.xml`, `verification.json`).
