# 실제 접촉으로 캔을 옮기는 NAMO 시뮬레이션

기존 `namo_sim`의 모의 제거 시연을 보존하고, 별도 `namo_manipulation`/`namo_rviz` 패키지로 Waffle Pi + OpenMANIPULATOR-X + 손목 D435i를 통합한다. 캔은 지름 5 cm, 높이 13.5 cm이다. 빈 캔의 질량은 실측값이 없어 15 g으로 가정했다. 그림의 **33초·38초는 실행 목표나 강제 대기 시간이 아니다.**

새 검증 영상: [레이저 표시·확대 지도·한 줄 자막 A/B](../.namo_manip/presentation/laser_view/demo_ab.mp4). 원본 YOLO를 사용했고 적치 후 출발부터 G까지는 1배속이다. [수치·출처 확인](../.namo_manip/presentation/laser_view/verification.json). [이전 공식 모델 영상](../.namo_manip/presentation/official/demo_ab.mp4)도 보존한다.

이 문서는 실행 방법과 구현 범위를 설명한다. 실제 성공·실패 여부는 `.namo_manip/runs/<run_id>/result.json`과 [검증 기록](docs/VALIDATION.md)으로 확인한다. 빌드 성공이나 계획 성공은 물리 파지·목적지 도착 성공과 다르다.

현재 버전은 공식 ROBOTIS Xacro의 팔·LiDAR·EE 배치를 복원하고 기존 EATED hand-eye 값을 자동으로 읽는다. 적치 후에는 현재 위치에서 G 하나를 목표로 2Hz 경로 갱신을 수행하며 중간 복귀점이 없다. 상세한 출처와 좌표는 [모델 설명](docs/MODEL_PHYSICS.md)을 참조한다.

## 보존 및 재사용

- 기존 `control_pkg`, `vision_pkg`, `joystick_pkg`, `eated_interfaces` 소스와 하드웨어 launch는 수정하지 않는다.
- `namo_sim`의 Nav2 액션, 시각이 있는 LiDAR/TF 처리, 정적 지도 A*, 장애물 분류, 정지 확인, 관측 기반 costmap clearing을 재사용한다.
- 기존 `vision_pkg`의 RGB-D 수학 함수와 hand-eye 장착 설정을 참조한다. 기존 전체 물체 운반 제어 노드는 별도 주행 명령을 내므로 함께 실행하지 않는다.
- 기존 `best.pt`를 그대로 읽는다. 현재 파일 경로는 `../best.pt`, 클래스 이름은 `coke`다. 재학습이나 정답 검출 주입은 하지 않는다.
- 새 팔 실행기는 관절 궤적과 양쪽 그리퍼 힘 제어, 실제 접촉을 사용한다. 객체 위치 이동 서비스, 삭제, 붙이기 플러그인은 사용하지 않는다.

## 빌드

현재 검증 환경은 Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11이다. Gazebo Sim/`ros_gz`용 launch가 아니다.

```bash
cd /home/user2/mobility_simulation/namo_ws/src
source /opt/ros/humble/setup.bash
source /home/user2/turtlebot3_ws/install/setup.bash
python3 namo_manipulation/tools/generate_robot.py
colcon --log-base .namo_manip/log build \
  --base-paths eated_interfaces vision_pkg control_pkg joystick_pkg namo_sim namo_manipulation namo_rviz \
  --build-base .namo_manip/build --install-base .namo_manip/install --symlink-install
source .namo_manip/install/setup.bash
python3 namo_manipulation/tools/prepare_tf2_runtime.py
```

빌드 산출물과 실행 기록은 `.namo_manip`에만 생성된다. 기존 `.namo` 시연은 별도로 유지된다.

현재 설치된 TF 0.25.23에서는 주행 중 TF callback의 교착을 실제 스택으로 확인했다. 마지막 명령이 계속 발행되는데 제어기가 사용하는 위치 시각이 멈추는 증상이었다. 위 준비 스크립트는 공식 Humble 0.25.24 바이너리 두 개를 SHA256으로 확인하고 `.namo_manip/runtime_fix`에 **압축 해제만** 한다. `run_demo.py`가 자식 프로세스에만 이 라이브러리를 적용하며 시스템 ROS 패키지는 변경하지 않는다. 관련 공식 기록은 [geometry2 #992](https://github.com/ros2/geometry2/issues/992), [#995](https://github.com/ros2/geometry2/issues/995)다. 실행마다 적용 버전과 해시가 `runtime_fix.json`에 남는다. 제어기 위치 시각이 다시 멈추더라도 속도 중재기는 즉시 주행을 차단한다.

## 실행·초기화·종료

아래 명령은 각자 Gazebo, 로봇, 센서, 미션 상태를 새로 시작한다. 정상 종료와 Ctrl+C 모두 해당 실행이 시작한 프로세스를 정리한다. ROS launch 자식이 별도 프로세스 그룹을 만들었을 때도 PID와 생성 시각으로 소유권을 확인하여 정리하고, 다른 실행의 프로세스는 종료하지 않는다. 기본 DDS domain은 44, Gazebo 포트는 11357이다. 동시에 다른 실험을 한다면 두 값 모두 변경한다.

```bash
# 먼저 장애물 없는 기본 주행
python3 namo_manipulation/scripts/run_demo.py --scenario A --no-obstacle --label baseline

# 긴 우회 통로: 경로/시간을 계산한 뒤 행동 결정
python3 namo_manipulation/scripts/run_demo.py --scenario A --perception yolo --gui --rviz --record

# 짧은 우회 통로: 같은 속도·팔 설정으로 비교
python3 namo_manipulation/scripts/run_demo.py --scenario B --perception yolo --gui --rviz --record

# 영상·로그·결과를 남기지 않고 화면에서 실행
python3 namo_manipulation/scripts/run_demo.py --scenario A --perception yolo --gui --rviz --camera closeup --no-save --domain 96 --port 11496

# 감지 후 정지만 검증 / 비용 계산까지만 검증
python3 namo_manipulation/scripts/run_demo.py --scenario A --stage detect
python3 namo_manipulation/scripts/run_demo.py --scenario A --stage plan

# 캔을 제거할 수 없다는 실험 조건
python3 namo_manipulation/scripts/run_demo.py --scenario B --immovable

# 생성 위치를 12 cm 이동: 미션은 해당 값을 받지 않고 실제 /scan으로 찾음
python3 namo_manipulation/scripts/run_demo.py --scenario A --stage detect --can-y 1.22
```

기구 동작을 재현하는 명시적 실행 명령은 `python3 namo_manipulation/scripts/run_demo.py --scenario A --perception sim_red --gui --rviz --record --camera closeup`이다. B는 `--scenario B --camera overview`로 실행한다.

기본 `yolo` 모드는 원본 모델이 `coke`를 실제 검출해야 진행한다. 기본 인식 자세에서 안정 관측이 부족하면 베이스 정지를 유지하고 팔 joint1을 기본값에서 -5°, +5°로 최대 두 번 바꿔 관측한다. 매 경로의 충돌 검사와 원본 신뢰도 기준 0.5·안정 관측 3회는 유지한다. 검출 실패 시 정지하고 `FRESH_CAN_RECOGNITION_FAILED`를 남긴다. 렌더링과 실물 학습 영상 차이를 분리해 기구 동작을 검증할 때만 `--perception sim_red`를 명시한다. 이 모드는 실제 RGB 색상과 depth를 사용하지만 **YOLO 인식 성공을 뜻하지 않는다.** 화면과 결과에 별도 모드가 표시되며 자동 전환되지 않는다.

영상은 `--gui --rviz --record` 실행 폴더의 `screen.mp4`에 저장한다. `--camera closeup`을 추가하면 Gazebo는 작업 공간을 확대하고 RViz는 전체 경로를 표시한다. 화면의 5 cm 캔은 전체 맵 보기에서 작으므로 집기 확인에는 손목 카메라 `/namo/can_debug` 영상과 접촉·물체 위치 검증 기록을 함께 본다. 실행 종료 뒤 같은 명령을 다시 실행하면 초기화된다. 진행 중 종료는 그 명령을 실행한 터미널에서 Ctrl+C를 누른다.

Gazebo의 파란 선은 LDS-02 ray 센서가 측정한 광선이다. 벽이나 캔에 닿는 끝점을 볼 수 있으며, 선이 닿았다는 사실과 NAMO가 여러 스캔으로 장애물을 확정했다는 사실은 구분한다. 콜라 캔이라는 분류는 이후 손목 카메라의 YOLO가 수행한다. Gazebo의 World/Insert/Layers 영역은 이 실행의 GUI 플러그인이 시작할 때 접는다. 사용자 전체 Gazebo 설정이나 다른 창은 변경하지 않는다.

`--gui --rviz`를 함께 지정하면 녹화 여부와 관계없이 두 창을 사용 가능한 데스크톱에 나란히 배치한다. RViz 지도는 실제 표시 영역에 맞춰 확대하며 수동 확대·이동 후에는 왼쪽 아래 **지도 전체 맞춤** 버튼으로 복귀할 수 있다. 새 편집 영상은 동작·재생 배속을 설명하는 한 줄 자막만 사용한다.

`--record`만 빼면 영상 녹화만 꺼지고 로그·결과는 계속 저장된다. `--no-save`는 영상 녹화와 관측 이미지·검증 probe 저장을 끄고, 실행에 필요한 설정과 노드 로그·결과를 임시 디렉터리에 둔다. 정상 완료·Ctrl+C·Python 예외 종료 뒤 임시 디렉터리를 삭제하며 `.namo_manip/runs/`에 새 실행 폴더를 만들지 않는다. 터미널과 RViz의 실시간 상태 표시는 유지한다. `--record`와 `--no-save`는 함께 지정할 수 없다. 강제 종료(SIGKILL)나 전원 차단은 정리 코드를 실행할 수 없으므로 임시 파일이 남을 수 있다. 기존 실행 기록을 삭제하는 옵션은 아니다.

`NAV_EXECUTION_FAILED status=6`은 Nav2 액션 중단 상태이며 그 자체는 원인이 아니다. 화면·녹화 실행에서 `compute_path_to_pose` 응답 확인 시간 초과를 재현했다. Humble의 `default_server_timeout` 단위는 ms이므로 20에서 1000으로 조정했다. 이는 전체 주행 제한 시간이 아니며 센서·위치·팔·경로 갱신의 안전 조건을 완화하지 않는다. 작업 접근 중 중단도 정지·새 스캔 4회·costmap 갱신·해당 작업 목표의 실제 Nav2 경로 검증 후에만 최대 두 번 재시도한다. 경로가 없거나 센서가 오래되면 계속 정지한다. 작업 목표 후보는 실행 전에도 실제 Nav2 경로로 검증한다. 원인 메시지는 RViz와 터미널에 표시하고, 실패 시 `--no-save`에서도 임시 파일 삭제 전에 Nav2의 최근 경고·오류를 출력한다.

## 경로와 판단

S=(-1.1, 0), P=(0, 0), G=(0, 3.4) m이며 짧은 통로 폭은 0.70 m다. 로봇 계획 반경은 0.28 m다. 캔은 정적 지도에 포함하지 않고 동적 물체로 둔다. 옆 적치 홈의 입구는 0.50 m로 계획상 로봇 지름 0.56 m보다 작아, 홈을 통한 우회가 지름길로 계산되지 않는다. 작업 위치는 관측한 캔에서 map y 방향으로 뒤로 0.34 m, x 방향으로 같은 위치인 후보이며 작업 방향은 map yaw 0°다. 공식 팔 장착점에 맞춰 측면 집기를 수행한다. P와 통로 중앙을 거쳐 작업 위치에 접근하고, 적치 후에는 현재 위치에서 G까지 한 목표로 주행하며 2Hz로 경로를 갱신한다. 중간 복귀 목표는 없다. 모든 접근 구간과 현재 위치에서 G까지의 후보 경로를 제거 비용에 포함한다. 고정 적치 중심은 기존 녹색 적치 공간 안의 map 좌표 (-0.40, 0.90) m다. 교차로 접근 뒤 최신 LiDAR/costmap으로 작업 목표를 보정하고, 정밀 목표의 근처 셀 대체는 허용하지 않는다. 팔 도달 여유를 확보하기 위해 작업 접근에만 Nav2 목표 오차 1.2 cm를 적용하고 도착 뒤 실제 위치도 확인한다. A 우회 세로 통로의 x는 9.5 m, B는 1.15 m다. 정밀 집기에 필요한 추가 정지·회전 시간을 포함한 동일한 판단식으로 선택이 달라지도록 실제 우회 길이를 다르게 만들었다. 지도와 Gazebo 벽은 `tools/generate_worlds.py` 하나로 생성한다.

LiDAR가 지도상 자유 공간의 새 점들을 연속 관측하면 정지한 실제 위치에서 두 경로를 계산한다. 카메라로 집기 가능성을 확인하기 전까지는 ‘실험 대상 단일 캔은 이동 가능’이라는 후보 가정이다. 제거를 선택해 작업 위치에 도착한 뒤 최신 RGB-D 인식과 충돌·도달 가능성을 다시 확인한다.

```text
T_remove = L_작업위치접근/v_eff + N_추가정밀목표 * T_정밀정렬 + T_카메라자세 + T_인식여유
           + T_팔궤적·그리퍼·정착 + L_적치후목적지/v_eff
T_detour = L_장애물유지우회/v_eff + T_switch
```

같은 정지 위치에서 계산하고, `T_remove + decision_margin < T_detour`이면 제거 후보를 선택한다. 제거 불가이면 우회하고 두 경로 모두 불가능하면 멈춘다. A/B 이름으로 선택을 지정하지 않는다. 팔 시간은 명목 작업 좌표에서 계획한 관절·직선 궤적 및 그리퍼 응답 추정치의 합이며 실제 파지 성공을 보증하지 않는다. 실제 궤적은 카메라 좌표로 다시 계산한다. `v_eff=0.12 m/s`는 센서 수신 병목 수정 후 B 실제 우회 1회(계획 길이 5.8322 m / 선택 후 47.0초 = 0.1241 m/s)를 반올림한 값이다. 일반 주행의 회전·정지가 포함되므로 `T_switch=0`이다. 작업 접근의 추가 정밀 목표 3개에는 각 16초의 추가 정렬·정지 비용을 적용한다. 적치 후 복귀 목표는 삭제했으므로 그 추가 비용도 제거했다. 이는 A 실측에서 팔 동작·일반 이동 예상 시간을 제외한 잔여 시간으로 보정한 근사값이다. A/B 공통 설정이며 광범위하게 검증한 예측 모델은 아니다. 인식 여유 2초는 가정값이다. 그리퍼는 일정 시간 기다린 뒤 성공 처리하지 않고 액션 결과와 양쪽 접촉을 확인한다.

REMOVE 순서는 **작업 위치 접근 → 정지 확인 → 카메라 자세 → 연속 인식 → 벌리기 → 접근 → 닫기 → 양쪽 접촉 확인 → 들어 올리기 → 옆 공간으로 운반 → 내리기 → 놓기 → 손 빼기 → 팔 접기 → LiDAR와 costmap 통로 확인 → 주행 재개**다. 주행은 미션 허가, 최신 제어기 위치 정보, 실제 팔 주행 자세와 잠금 해제가 모두 확인될 때만 허용된다.

## RViz와 터미널 읽기

`namo_rviz/StatusPanel`이 RViz 왼쪽에 한국어 상태를 표시한다. 표시 항목을 자세히 보려면 RViz 메뉴 `Panels → Add New Panel → Displays`로 추가할 수 있다. 발표 화면은 상태 패널에 공간을 배정한다. `Displays`의 초록 OK는 해당 표시 항목이 정상이라는 뜻이며 미션 성공을 뜻하지 않는다. 미션 성공은 NAMO 창의 ‘목적지 도착’과 결과 파일의 `execution_completed=true`로 확인한다.

| 표시 | 의미 |
|---|---|
| 현재 / 팔 상태 | 미션 단계와 실제 팔 동작 단계 |
| 베이스 주행 잠금 / 허용 | 속도 중재기의 실제 허용 상태와 이유 |
| 치우고 직진 / 우회 예상 | 판단 당시 남은 예상 총시간, 카운트다운 아님 |
| 실제 경과 | 시뮬레이션 시각으로 측정한 미션 경과 |
| 파랑 선 | 대상 캔을 치웠다는 가정 아래 계산한 후보 경로 |
| 주황 선 | 캔을 유지한 실제 Nav2 costmap의 우회 후보 |
| 보라 선 | 현재 위치에서 목표까지 계속 갱신하는 실제 Nav2 경로 |
| 초록 점 | 실제 LiDAR 반사점(벽 포함) |
| 빨강 점 / 노랑 점 | 감지 당시 캔 위치 기록 / 판단을 위해 정지한 위치 |
| S / P / G / PLACE | 출발 / 판단 부근 / 목적지 / 캔 적치 위치 |
| YOLO / SIM_RED | 실제 학습 모델 인식 / 명시적인 색상·깊이 실험 모드 |

`status.log`와 터미널에도 같은 상태를 출력한다. `/namo/status`는 최신 전체 JSON 상태이며 `/namo/physical_removal_status`는 팔 단계, 예상 시간, 실제 접촉 증거와 오류를 제공한다. `/namo/can_observation`은 검출 신뢰도·안정성·변환 좌표·거절 이유를 제공한다. 주행용 노드는 Gazebo 객체 정답 위치를 입력받지 않는다. `navigation_probe.jsonl`의 `validation_only_world_pose`와 `validation_only_can_world_pose`는 사후 검증용이다. Gazebo world, wheel odom, AMCL map은 서로 다를 수 있다. 검증기는 동일 시점의 로봇 world 자세로 카메라의 base_link 좌표를 world로 옮겨 비교하고, 목표 오차는 위치 추정 기준과 world 기준을 나누어 기록한다.

## 결과와 설정

주행·회전 구간의 폭은 0.90 m이고 캔을 집는 짧은 세로 통로는 0.70 m다. 지도 해상도와 LiDAR 벽 관측, 위치 추정 오차가 겹쳐 회전 구간에서 정지하던 것을 실제 벽·지도 양쪽의 여유로 보완했다. 캔이 있는 짧은 통로를 통과할 수 없다는 경로 검증은 유지한다.

독립적인 물리 집기 검증은 `python3 namo_manipulation/scripts/run_arm_demo.py`로 실행한다. Domain 43에서 새 장면을 만들고, 명시적인 SIM_RED 관측 → 집기 → 운반 → 놓기를 시험한다. `--perception yolo`로 원본 모델 검출을 필수로 지정할 수 있다. Gazebo 정답 좌표는 검증용 probe만 기록하며 팔 목표는 실제 RGB-D 관측에서 받는다.

각 실행 폴더에 `result.json`, `result.csv`, `events.jsonl`, `trajectory.csv`, 설정 스냅샷, 노드별 로그를 저장한다. 선택 확정부터 목적지 도착까지의 시간과 전체 시간을 구분하고, 시뮬레이션 초와 실제 벽시계 초를 따로 기록한다. 실패한 실행의 경과 시간은 완료 시간으로 해석하면 안 된다. 실행하지 않은 후보의 실측 시간은 만들지 않는다.

| 파일 | 조정할 내용 |
|---|---|
| `config/mission.yaml` | 유효 속도, 판단 여유, 작업 접근 위치, 적치 위치, 단계 제한 시간 |
| `config/arm.yaml` | 작업 속도, 들기 높이, 그리퍼, 카메라 자세와 접촉 검사 |
| `config/perception.yaml` | 원본 모델 경로, 클래스, 신뢰도, RGB-D 토픽, 안정화, 바닥 가정 |
| `config/nav2.yaml` | 실제 footprint 반경, 속도, 지도·LiDAR costmap, 목표 도착 허용 오차 |
| `config/controllers.yaml` | 팔·양쪽 그리퍼의 Gazebo 제어 설정 |
| `rviz/manipulation.rviz` | 왼쪽 상태 패널과 지도·경로·센서 화면 |

공식 URDF 출처, 팔 앞 LiDAR 배치, EATED hand-eye와 물리 모델 가정은 [MODEL_PHYSICS.md](docs/MODEL_PHYSICS.md)에 상세히 기록한다. 바닥이 평평하고 캔이 세워져 있는 실험이다. 13.5 cm 캔과 약 13.2 cm 레이저 높이는 여유가 작아 실물에서 반드시 측정해야 한다.

팔의 계획은 알려진 실험실 구조에서 4자유도 IK와 보수적인 링크 캡슐/벽 충돌 검사를 사용한다. 임의 환경에 대한 MoveIt 메시 충돌 계획을 구현했다고 주장하지 않는다. 실물 전환 때에는 정확한 hand-eye 보정, 카메라 깊이 오차, 실제 그리퍼 접촉/파지 검증, 질량·무게중심과 들기 하중, 하드웨어 비상정지, MoveIt 충돌 모델을 다시 확인해야 한다. 이 launch는 하드웨어 드라이버를 시작하지 않는다.

## 핵심 로직 검증

```bash
source .namo_manip/install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="namo_sim:namo_manipulation:vision_pkg:$PYTHONPATH" \
  python3 -m pytest -q namo_sim/test namo_manipulation/test
```

검증은 경로 유효성, 두 비용의 선택 규칙, 캔/적치 홈 통과 가능성, IK/충돌, 센서 변환과 안정성, 팔 상태·관절 신선도·베이스 이동 차단을 포함한다. 물리 파지와 실제 주행 결과는 별도 Gazebo 실행으로 검증한다.

완료된 녹화 A/B를 1~2분 발표 영상으로 합치려면 다음 명령을 쓴다. 주행 3배속, 팔 1.25배속을 영상에 명시하며 원본은 보존한다.

```bash
python3 namo_manipulation/scripts/make_demo_video.py \
  .namo_manip/runs/<A_성공실행> .namo_manip/runs/<B_성공실행> \
  .namo_manip/presentation/demo_ab.mp4
python3 namo_manipulation/scripts/validate_recorded_run.py .namo_manip/runs/<실행>
```

이전 전방 마운트·중간 복귀점 버전의 보존 영상(SIM_RED): [A/B 1분 59초](../.namo_manip/presentation/demo_ab.mp4). 집기 구간은 1.25배속, 주행은 3배속이며, 같은 녹화 화면의 집기 영역을 확대해 함께 표시한다. [최종 수치 CSV](../.namo_manip/presentation/results.csv)에서 예상 시간과 실제 시간을 비교할 수 있다.


## 추가 파일의 역할

| 파일/디렉터리 | 역할 |
|---|---|
| `namo_manipulation/mission.py` | LiDAR/TF 관측 큐, 정지 확인, 시간 비교, 작업 접근·현재 위치에서 G까지 연속 주행과 Nav2 상태 관리 |
| `namo_manipulation/can_perception.py` | 원본 YOLO 또는 명시적 SIM_RED, 동기화 RGB-D 원기둥 중심과 안정성 검사 |
| `namo_manipulation/arm_executor.py` | PREPARE/REMOVE/STOW 서비스, 팔·양쪽 그리퍼 제어와 실제 접촉 확인 |
| `namo_manipulation/arm_kinematics.py` | 4자유도 FK/IK, 직선/관절 궤적 및 보수적 충돌 검사 |
| `namo_manipulation/drive_gate.py`, `drive_safety.py` | 주행 허가·팔 상태·센서/제어기 신선도에 따른 실제 속도 중재 |
| `namo_manipulation/status_display.py` | RViz와 같은 상태를 한국어 터미널 로그로 출력 |
| `../namo_rviz/src/status_panel.cpp` | RViz 왼쪽 한국어 상태 패널 |
| `tools/generate_worlds.py`, `maps/`, `worlds/` | 동일 원본에서 만든 실제 벽과 정적 지도 |
| `tools/generate_robot.py`, `models/` | 설치 자산을 보존하는 별도 물리 모델·캔·손목 카메라 |
| `launch/`, `scripts/run_demo.py` | 하드웨어와 격리한 실행·초기화·종료·녹화 |
| `scripts/validate_recorded_run.py`, `make_demo_video.py` | 읽기 전용 사후 물리 검증 및 배속을 명시한 발표 영상 |

이전 전방 마운트 버전에서는 원본 `best.pt` YOLO도 별도 통합 실행에서 실제 집기·적치·목적지 도착까지 성공했다. 현재 공식 배치의 인식 결과는 최신 검증 기록을 따른다.
성공값은 confidence 약 0.505로 문턱에 가까우며, 다른 시점의 영상에서 낮은 값도 관측되었다.
검출 실패 시 자동 색상 인식으로 바뀌지 않는다. 상세 근거와 제한은 [인식 검증](docs/PERCEPTION.md)에 기록했다.

YOLO는 별도 전체 완주에 성공했지만 직후 반복 녹화에서는 인식 실패로 안전 정지했다. 반복 인식 안정화는 남아 있으며, 제공 영상의 SIM_RED 표시를 YOLO 성공으로 해석하지 않는다.
