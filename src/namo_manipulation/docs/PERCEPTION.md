# 손목 RGB-D 캔 관측과 검증 범위

`can_perception`은 실제 카메라 토픽과 같은 시각의 TF만 사용한다. Gazebo
`model_states`/`link_states`/`GetEntityState`는 위치 계산에 사용하지 않는다.
기본 모드는 원본 `best.pt`의 **YOLO**이며, **SIM_RED**는 명시적으로 선택하는
시뮬레이션 색상 분할 실험이다. YOLO 실패 시 SIM_RED로 자동 전환하지 않는다.

## 원본 모델과 기존 코드 재사용

- 가중치: `/home/user2/mobility_simulation/namo_ws/best.pt`.
- SHA256: `d0892d275d9795595d0808136c692dbec5652b08e6e451589ebf01971b68ab08`.
- 실제 로드 결과: YOLO11n, 클래스 `airpod`, `coke`, `tissue`, `vaseline`.
- 기본 요구 조건: 클래스 `coke`, confidence ≥ 0.5. 진단 스크립트는 0.01 이상
  예측도 저장하지만, 이를 성공 판정에 사용하지 않는다.
- 학습 설정의 데이터 경로는 `/home/sh/yolo_ws/datasets/coke_tissue_airpod_vaseline/data.yaml`이며
  현재 환경에는 해당 데이터셋이나 검증 가능한 원본 학습 사진이 없다.
- 기존 `vision_pkg.vision_utils`의 `Detection`, `deproject_pixel`,
  `select_target_detection`을 재사용한다. 기존 패키지 파일은 수정하지 않는다.
- 설치된 Humble `cv_bridge`와 사용자 NumPy 2 사이 ABI 오류가 있어, 새 어댑터는
  ROS Image의 encoding/endianness/stride를 검사한 뒤 직접 배열로 읽는다.

## 센서 및 출력 계약

입력은 `/realsense/d435i/color/image_raw`,
`/realsense/d435i/aligned_depth_to_color/image_raw`,
`/realsense/d435i/color/camera_info`이다. RGB와 depth는 같은 optical frame,
640×480 등록 영상이어야 한다. 비영 왜곡 계수는 거부한다. 실제 Gazebo 검증에서는
RGB8/32FC1, fx=fy≈462.2663, cx=320.5, cy=240.5를 확인했다.

원본 launch에서 가져온 nominal 손목 마운트는
`end_effector_link → d435i_link = (-0.065, 0.0195, 0.052)m`, RPY=0이다.
이는 실제 측정으로 입증된 hand-eye calibration 값이 아니다. 별도의 보정 결과
파일은 발견하지 못했다. 물리 장비 적용 전에 실제 카메라 보정과 마운트를 확인해야 한다.

출력 `/namo/can_pose`는 **base_link 좌표계의 캔 중심**이다. grasp 위치가 아니다.
같은 이미지 timestamp의 `/namo/can_observation` JSON에서 `valid=true`,
`stable=true`이고 pose stamp가 일치할 때만 후속 실행기가 사용해야 한다.
기본 안정 조건은 서로 15mm 이내인 연속 관측 3회다. 팔의 grasp 높이 조정은
별도 실행기에서 수행한다. `/namo/can_debug`는 실제 RGB와 현재 인식 모드를 표시한다.

지원하는 물체 가정은 평평한 바닥에 똑바로 서 있는 지름 0.05m, 높이 0.135m 캔이다.
고정 `odom`의 z=0을 바닥으로 사용하고, 이미지 아래 실제 깊이로 바닥 가정을
확인한다. 베이스가 기울 수 있으므로 움직이는 `base_footprint`를 바닥 기준으로
사용하지 않는다. 실제 full-odometry TF를 적용한 바닥 투영 오차는 아래 검증
프레임에서 약 0.3mm였다. 임의 경사면/탁자/누운 캔에는 해당 프로파일을 그대로
사용할 수 없다.

최종 옆면 위치 추정은 bbox 안의 실제 등록 depth를 고정 바닥 좌표로 투영하고, 캔 옆면 높이 구간의 XY 점들에 원을 맞춘다. 측정 반경이 25±4 mm, 잔차 95백분위가 2 mm 이하, 관측 원호가 60° 이상이어야 허용한다. 라벨의 흰 줄 때문에 빨간 영역이 분리되어도 bbox 중심이 캔 중심이라는 가정을 하지 않는다. 이전 단일 픽셀+반지름 방식은 이 경우 약 2 cm 치우칠 수 있어 실행 경로에서 제거했다.
SIM_RED의 빨간 옆면이 초승달 모양이라 중심 픽셀 깊이를 사용할 수 없는 경우,
red seed 주변의 실제 depth에서 상단 높이의 원형 경계를 검증한다. 관측 원은
반경 0.025±0.004m, 최대 잔차 3mm 이하이고 270° 이상의 경계 범위를 보여야 한다.
반쪽 원이나 타원은 거부한다. 이 방법도 관측 기반이며 simulator pose를 사용하지 않는다.
보정 방식은 `correction` JSON 필드에 명시한다.

## 실제 검증 증거

증거 위치는 저장소 루트의 `.namo_manip/` 아래다.

| 증거 | 실제 결과 | 해석 |
| --- | --- | --- |
| `perception_check/` | 무로고 빨간 캔 전체 영상, YOLO 0.01 이상 예측 없음 | 원본 모델이 단순 원통 시각 도메인을 인식하지 못함 |
| `perception_odom_check/` | 고정 odom 바닥 투영 오차 약 0.6~0.8mm, 캔 하단 crop | TF 일관성 확인; 이 영상만으로 인식 성능 판정 불가 |
| `perception_65deg/` | 누적 베이스 이동으로 캔이 시야 밖 | 인식 실패율 평가 대상 아님 |
| `perception_label_prepared/` | 실제 라벨 적용 캔 영상에서 `airpod=0.884`, coke 없음 | strict YOLO는 오분류를 성공으로 받지 않음 |
| `perception_label_prepared/lid_depth_validation.json` | 실제 저장 RGBD에서 관측 반경 0.02492m, 최대 잔차 0.207mm, 바닥 오차 0.277mm | 오프라인 기하 검증이며 fresh 조작 입력이 아님 |
| `perception_live/live.log` | live SIM_RED가 stable 중심을 발행한 뒤 팔 이동 중 crop/미검출을 거부 | 센서 기반 관측 경로 동작; fixture 팔 시험의 성공 여부와 별도 |

`cola_label.svg/png`, `cola_side.dae`는 코드로 만든 시각적 근사이다. 공식 제품
라벨이나 실제 촬영 사진이 아니다. 라벨 적용은 visual만 바꾸며 충돌 형상, 크기,
질량이나 인식 결과를 수정하지 않는다. `cola_label_metadata.json`에 출처를 기록했다.
최종 주행 작업 자세(카메라 관절 yaw -22°, tool pitch 44°)의 실제 저장 영상도 원본 모델로 다시 검사했다. `yolo_main_recognition_pose/evidence.json`에서 `coke=0.184`였으며 기준 0.5를 통과하지 못했다. 이 프레임에서는 strict YOLO가 통과하지 못했다. 이후 별도 통합 실행 `runs/20261117_035403_final_strict_yolo`에서는 실제 `coke` confidence 약 0.505, 연속 안정 관측 3회를 통과해 원본 YOLO → 깊이 원기둥 중심 → 물리 집기·적치 → G 도착까지 성공했다. 기준 0.5와 원본 가중치는 유지했으며 SIM_RED 전환은 없었다. 시점에 따라 confidence가 크게 달라지고 성공값도 경계에 가까우므로 안정적인 인식률을 보증하지 않는다.

## 실행과 재현

`config/perception.yaml`의 기본은 `perception_mode: yolo`이다. 색상 실험은
실행 인자로 `-p perception_mode:=sim_red`를 명시해야 한다. 모든 상태와 로그에
SIM_RED가 표시되며 YOLO confidence 대신 null을 쓴다.

CPU 경합을 줄이기 위해 모델 초기화 시 PyTorch `cpu_threads=2`, inter-op=1을
설정한다. Ultralytics의 문자열 `device='cpu'`가 스레드 수를 재설정하므로
`torch.device('cpu')`를 전달한다. 실제 카메라 프레임 재생에서 최종 thread 수 2를
검증했으며, 원본 모델 예측값을 변경하지 않는다.

`scripts/verify_camera_yolo.py --output <폴더>`는 실제 RGBD와 같은 시각의
optical/base_link→odom TF, 가중치 해시, 모든 진단 예측, 실제 영상과 annotation을
저장한다. 팔/베이스 명령을 보내지 않는다. 노드 evidence에는 상태 JSONL,
마지막 stable JSON과 원본 RGB를 보존한다. 기존 단일 `observation.json`이 이후
관측으로 덮일 수 있으므로 성공/실패 순서 검토에는 JSONL을 사용한다.

순수 기하/이미지 해석 테스트 20개는 통과했다. 학습 모델의 실제 검출, 카메라
보정 정확도, 물리 파지 성공을 단위 테스트가 대신 입증하지는 않는다.

## 원본 YOLO 통합 성공 증거

`runs/20261117_035403_final_strict_yolo/result.json`의 `camera_observation`은
mode=YOLO, class_name=coke, correction=observed_depth_cylinder_side_fit이다.
`arm_remove_result.json`의 target_source=RGBD_YOLO이며, 실제 양쪽 접촉·들기·분리를 확인했다.
`validation.json`에서 캔 상승 5.21 cm, 전체 캔이 적치 패드 안에 있음, 부착/순간이동 없음이 확인된다.
목표 도착까지의 전체 성공이며, 저장 사진 한 장을 재추론한 결과와 구분한다.
앞선 0.184/미검출 프레임도 그대로 보존했다. 한 번의 통합 성공을 반복 인식 성능 평가로 해석하지 않는다.


직후 반복 녹화 `runs/20261117_035817_A_yolo_record`에서는 30초 wall timeout 내
안정적인 YOLO 관측을 확보하지 못해 `FRESH_CAN_RECOGNITION_FAILED`로 정지했다.
REMOVE 서비스는 호출하지 않았고 마지막 실제 속도 명령은 0이었다.
따라서 현재 가중치/렌더링 조합의 반복 인식 안정성은 확보되지 않았다.
배포한 A/B 영상은 SIM_RED로 물리 동작을 검증한 별도 성공 실행이며,
원본 YOLO 전체 완주 성공은 위 headless 실행 로그로 제공한다.

## 선택한 영상과 집기 입력의 연결

팔 실행기는 최근 16개 관측 메타데이터를 보관하고, 미션이 선택한 PoseStamped와 sec/nanosec가 **정확히 같은** 항목만 사용한다. 같은 frame과 center_xyz인지도 검사한다. 최신 YOLO 결과가 다음 영상으로 갱신되어도 이미 선택한 유효 관측과 혼동하지 않는다. 기존 2초 목표 신선도와 valid/stable 검사는 유지하며, 짝이 없거나 좌표가 다른 입력은 거부한다.

공식 배치의 `runs/20261117_051720_official_yolo_paired`에서 원본 `best.pt`, class=coke, confidence=0.6413, 안정 관측 3회로 실제 집기·적치·G 도착을 완료했다. 인식 기준 0.5를 낮추거나 SIM_RED로 전환하지 않았다. 실제 깊이와 TF를 이용한 캔 중심의 사후 XY 오차는 약 1.09mm였다. 이 값은 단일 시뮬레이션 실행의 관측 정합 결과이며 실물 인식 정확도나 반복 성공률을 보증하지 않는다.
