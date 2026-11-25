# EATED NAMO 중간 점검 시뮬레이션

ROS 2 Humble / Gazebo Classic 11 / TurtleBot3 Waffle Pi에서 실제 구동계와 Nav2로
이동하고, `/scan`과 TF로 통로의 새 장애물을 감지한 뒤 제거·우회 예상 시간을
비교하는 독립 패키지다. 기존 `control_pkg`, `vision_pkg`, `joystick_pkg`,
`eated_interfaces`는 수정하지 않는다. 기존 소스·설치 환경 조사는
[docs/ENVIRONMENT.md](docs/ENVIRONMENT.md), 실행별 최종 검증 결과는
[docs/VALIDATION.md](docs/VALIDATION.md)를 확인한다.

**제거 동작은 모의 제거(MOCK REMOVE)**다. 실제 정지를 확인하고 설정한 시간만큼
기다린 뒤 Gazebo 서비스로 물체를 적치 공간에 옮긴다. 팔 접촉·파지 물리,
YOLO 재학습, 실제 D435i 스트림은 이 시연에 포함하지 않는다.
LiDAR는 물체 발견과 관측 위치 추정을 담당하며, 이동 가능 여부는
`removable` 실험 가정이다. 콜라 캔 인식이나 파지 가능성 판정 결과가 아니다.

## 빌드와 기본 실행

새 터미널에서 다음을 실행한다. 빌드·설치·로그는 작업 공간의 `src/.namo`에
만들어 기존 하드웨어 워크스페이스 설치본과 분리한다.

```bash
cd /home/user2/mobility_simulation/namo_ws/src
source /opt/ros/humble/setup.bash
colcon --log-base .namo/log build --base-paths namo_sim \
  --build-base .namo/build --install-base .namo/install --symlink-install

# 1. 장애물 없는 실제 Nav2 기본 주행: S → P → G
python3 namo_sim/scripts/run_demo.py --baseline

# 2. 실제 LiDAR 감지·취소·정지까지만 확인
python3 namo_sim/scripts/run_demo.py --scenario A --stage detect

# 3. 경로 길이·예상 시간 계산까지만 확인
python3 namo_sim/scripts/run_demo.py --scenario A --stage plan

# 4. A/B 전체 시연: 각각 별도로 실행
python3 namo_sim/scripts/run_demo.py --scenario A --gui --rviz
python3 namo_sim/scripts/run_demo.py --scenario B --gui --rviz
```

runner가 `.namo/install/setup.bash`를 읽고 고유 실행 폴더를 출력한다.
`--stage full`이 기본값이다. GUI가 필요 없으면 `--gui --rviz`를 생략한다.
기본 제한 시간은 실제 벽시계 420초이며 `--timeout 600` 등으로 조절할 수 있다.
`detect`/`plan` 성공은 해당 단계 확인을 뜻하며 목적지 완주와 구분한다.

종료는 실행 터미널에서 **Ctrl+C**다. runner는 자신이 만든 프로세스 그룹만
정리한다. 정상 완료와 제한 시간 초과에도 정리한다. 같은 명령을 다시 실행하면
Gazebo, 로봇·물체 위치, `/clock`, AMCL, 미션, RViz 표시와 로그가 새로 시작된다.
실행 중 `/reset_simulation`만 호출하는 방식은 지원하지 않는다.

기본 통신 영역은 `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`,
`GAZEBO_MASTER_URI=http://127.0.0.1:11355`이다. 한 번에 runner 하나를 실행한다.
별도 실험을 병렬 실행하려면 `--domain`과 `--port`를 모두 다르게 지정한다.
기존 하드웨어 control/joystick launch는 이 시연에서 시작하지 않는다.

## 맵과 물체

| 항목 | 설정 |
| --- | --- |
| 시작 S | `(0, -2)`, yaw `π/2` |
| 관측 목표 P | `(0, 0)`; 도착 자체는 발견 이벤트가 아님 |
| 목적지 G | `(6, 0)` |
| 짧은 통로 | P에서 G로 연결되는 y=0 통로 |
| A 우회 통로 | `(0, 5) → (6, 5)` 부근 |
| B 우회 통로 | `(0, 1.8) → (6, 1.8)` 부근 |
| 통로 유효 폭 | 1.4 m |
| 기본 장애물 중심 | `(2.4, 0, 0.3)`; 감지기에는 이 좌표를 전달하지 않음 |
| 장애물 크기 | 0.45 × 1.0 × 0.6 m 상자 |
| 적치 공간 | `(-2, -2, 0.3)`, 주행 통로 외부 |
| 지도 해상도 | 0.05 m/cell |

실제 고정 벽이 시작점에서 장애물을 가린다. `tools/generate_worlds.py`의 동일한
직사각형 목록으로 Gazebo 충돌 벽과 정적 지도 PGM을 생성하므로 시야를 가리는
벽과 지도 장애물이 일치한다. 이동 대상은 정적 지도에 포함하지 않는다.
필요하면 생성기를 수정한 뒤 다음 명령으로 A/B 전체 자산을 다시 생성하고 빌드한다.

```bash
python3 namo_sim/tools/generate_worlds.py
```

Waffle Pi 모델과 구동 플러그인은 설치된 ROBOTIS 모델을 재사용한다.
`tools/generate_robot.py`가 만든 전용 SDF에서 LiDAR 센서 z만 0.121→0.131 m로
보정하여 URDF의 `base_footprint → base_link → base_scan` TF와 맞췄다.
SDF joint pose는 조인트 앵커 위치이며 센서 XY 위치가 이중 가산되는 구조가 아니다.
중간 점검 상자는 이 스캔 높이와 교차하는 대체 모델이며 실물 콜라 캔과 크기가 다르다.

Nav2와 제거 후보 A*의 원형 반경은 모두 **0.28 m**다. 베이스 충돌 상자의
외접 크기와 안전 여유를 포함한다. A*는 점유 셀 중심 거리에서 셀 면적을 보수적으로
반영하므로 0.05/√2 m의 셀 반대각 여유도 적용한다. 미지 영역과 지도 밖은
통행 불가이며 대각선으로 벽 모서리를 통과하지 않는다.
상자 양쪽의 남은 폭은 각각 0.2 m여서 베이스가 옆으로 통과할 수 없다.

## 감지·정지·계획·실행

`mission.py`는 센서 QoS로 실제 `/scan`을 읽고 유효 거리만 메시지 시각의 TF로
map 좌표에 변환한다. 정적 벽 근처 0.13 m는 지도·위치 추정 오차로 보고 제외한다.
남은 점은 0.16 m 연결 거리로 묶고, 기본 5점 이상·경로 횡방향 폭 0.25 m 이상·
연속 3회 관측으로 확인한다. 명목상 P 좌표나 타이머로 발견을 만들지 않는다.
확인한 물체가 일시적으로 보이지 않아도 감지 정보를 유지한다.

장애물 확인 시 Nav2 목표를 취소하고 `/odom`으로 정지를 확인한다.
정지한 실제 로봇 pose가 두 후보의 공통 출발점이다.

- **제거 후보**: 정적 지도 복사본에서 A*를 계산한다. 고정 벽과 가림막은 유지하며
  함께 관측된 다른 새 장애물은 추가 점유로 보존한다. 실제 주행 costmap을 지우지 않는다.
- **우회 후보**: `/scan`의 장애물이 반영된 global costmap을 확인한 후 Nav2
  `ComputePathToPose(use_start=true)`를 요청한다. 시작·끝점, 빈 경로, 관측 장애물
  및 치명 비용과의 교차를 검증한다. 계획 실패를 직선거리로 대체하지 않는다.

계산은 두 경로의 연속 위치점 사이 거리를 합산한다. A/B 차이는 실제 고정 구조와
경로 길이이며, 시나리오 이름은 기록용이다. 선택을 코드로 지정하지 않는다.
선택 후 이동은 Nav2가 수행한다. 전용 BT는 실패 시 액션을 중단하며 자동 전체
costmap 지우기나 복구 주행으로 NAMO 판단을 건너뛰지 않는다.

REMOVE는 `MOCK REMOVE` 상태에서 설정한 시뮬레이션 시간 동안 정지한 뒤
`/gazebo/set_entity_state`로 상자를 적치 공간에 이동한다. 이 모델 이름과 서비스는
모의 제거 실행기만 사용하며 감지·판단에 simulator pose를 공급하지 않는다.
서비스 성공 뒤에도 새 LiDAR의 관통 광선 5개 이상과 연속 확인, 새 global costmap의
통로 비움을 확인한 후 출발한다. 화면에서 사라짐이나 관측 부재만으로 성공 처리하지 않는다.
DETOUR는 제거 요청 없이 기존 장애물을 유지하고 우회한다.

원본 LDS 스캔의 각도 간격 때문에 이미 빈 통로에 costmap 셀 몇 개가 남는 현상을 실제로
확인했다. `scan_clearing`은 인접 광선의 거리 차이가 작은 구간만 네 배로 샘플링하며,
두 광선 중 더 짧은 거리에서 2cm를 뺀 범위까지만 자유 공간으로 사용한다. 불연속·무효 구간을
넘어 보간하지 않는다. 이 `/scan_clearing` 입력은 양쪽 costmap의 **clearing 전용**이고,
장애물 표시와 감지는 원본 `/scan`만 사용한다. 센서 사이 각도 영역이 연속적이라는 근사를
명시한 격자 광선 처리이며 모델 위치나 제거 이벤트로 costmap을 지우지 않는다.
제거 후에는 새 원본 스캔의 통과 광선, 대상 영역의 lethal 셀 소멸, Nav2의 짧은 경로를 모두 확인한다.

주행 명령은 `controller_server → /cmd_vel_nav → velocity_gate → /cmd_vel`로 흐른다.
판단·제거 중 게이트를 닫는다. 게이트는 기본 닫힘이며 미션 heartbeat 또는 Nav2
명령이 오래되면 실제 벽시계 타이머로 0 속도를 발행한다.

목표는 위치 도착이며 도킹 방향은 요구하지 않는다. Nav2의 XY 도착 허용 오차는 0.12m,
방향 허용 오차는 π rad로 둔다. 실제 도킹/파지 접근으로 전환할 때는 필요한 목표 방향과
최종 회전 제어를 별도로 검증해야 한다.

## 시간 모델과 주요 파라미터

`config/mission.yaml`에서 다음 식을 설정한다. 현재 값은 **실측 보정 전 가정값**이다.
모의 제거 성공률은 p=1이며, 관측되지 않은 전역 장애물까지 포함하는 최적성을 주장하지 않는다.

```text
T_remove = mock_remove_seconds + L_clear / v_eff
T_detour = switch_seconds + L_detour / v_eff

제거 가능하고 T_remove + decision_margin < T_detour → REMOVE
두 경로가 가능하고 위 조건이 아니면                  → DETOUR
제거 불가                                         → 가능한 DETOUR
우회 경로가 실제 없고 제거 후 경로가 유효             → 가능한 REMOVE
둘 다 불가 / 필요한 데이터 없음                     → STOP + 원인 기록
```

| 파라미터 | 기본값 | 조정 기준 |
| --- | --- | --- |
| `v_eff` | 0.20 m/s | 일반 회전·감속을 포함한 평균 이동 속도 가정; 실제 주행으로 추후 보정 |
| `mock_remove_seconds` | 22.0 s | 실제 모의 제거 대기 시간과 비용 모델에 같은 값 적용 |
| `switch_seconds` | 0.0 s | 주행 전 추가 전환 지연만 포함; 일반 회전 시간을 중복 가산하지 않음 |
| `decision_margin` | 2.0 s | 작은 추정 차이에서는 우회 유지 |
| `removable` | true | 단일 실험 물체의 이동 가능 가정 |
| `robot_radius` | 0.28 m | Nav2 양쪽 costmap의 `robot_radius`와 함께 맞출 것 |
| `wall_tolerance` | 0.13 m | 벽 오인과 작은 물체 누락 사이 조정 |
| `min_points`, `persistence` | 5, 8 | 약1.4–1.6초의 실제 연속 관측; 코너의 부분 관측 확정 억제 |
| `ready_timeout` | 90 s | 준비 단계의 벽시계 제한 |
| `action_timeout` | 180 s | 주행 액션의 벽시계 제한 |
| `planning_timeout` | 12 s | 계획 요청·응답의 벽시계 제한 |
| `clear_timeout` | 15 s | 제거 후 관측 확인의 벽시계 제한 |

`mock_remove_seconds`는 YAML의 `mission`과 `mock_remover` 두 항목을 함께 수정한다.
미션이 실행기 상태의 실제 값을 읽어 불일치하면 `MOCK_DURATION_MISMATCH`로 중단한다.
A/B의 속도·제거 시간·마진을 다르게 설정하여 선택을 맞추지 않는다.
runner는 각 실행의 최종 설정을 `parameters.yaml`에 저장한다.

## 추가 점검과 핵심 테스트

```bash
# 벽만 있는 맵에서 거짓 제거 판단 없이 완주
python3 namo_sim/scripts/run_demo.py --scenario A --no-obstacle

# 조금 이동한 물체의 LiDAR 관측 위치 확인: 원래 2.4 → 2.65 m
python3 namo_sim/scripts/run_demo.py --scenario A --stage detect --obstacle-x 2.65

# 제거 불가 가정에서도 가능한 우회를 선택
python3 namo_sim/scripts/run_demo.py --scenario B --immovable

# 핵심 로직 테스트: 설치된 외부 pytest plugin 버전 충돌을 피함
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=namo_sim \
  python3 -m pytest -q namo_sim/test
```

테스트는 A* 점유·원점·코너 통과·동적 복사본, 비용 선택 규칙, 벽/새 물체 구분,
관측 위치 변화, 연속 확인, 추가 물체, 관측 없는 제거 완료 방지 등을 다룬다.
GUI·녹화·실물 성공은 단위 테스트 통과와 별도로 판단한다.

오류 발생 시 `mission.log`, `launch.log`, `remover.log`, `result.json`을 확인한다.
예를 들어 `PLANNER_UNAVAILABLE`, `PLAN_*_TIMEOUT`, `PLANNER_EMPTY_PATH`,
`PLANNER_ABORTED_OR_ERROR`, `GLOBAL_COSTMAP_MISSING_OBSTACLE`, `REMOVE_FAILED`,
`CLEARANCE_UNCONFIRMED`는 서로 다른 원인을 표시한다. 오류를 정상 경로로 감추지 않는다.
시뮬레이션 시간이 뒤로 이동하면 미션을 중단하고 관련 시뮬레이션 경과 시간을
유효한 음수 측정값으로 기록하지 않는다.

## 로그, CSV와 화면

실행 폴더는 `.namo/runs/YYYYMMDD_HHMMSS_<label>/`이다. `--label demo_A`처럼 지정할 수 있다.

| 파일 | 용도 |
| --- | --- |
| `launch.log`, `mission.log`, `remover.log` | 시뮬레이터/Nav2, 미션, 모의 제거 로그 |
| `parameters.yaml` | 해당 실행에서 사용한 최종 파라미터 |
| `events.jsonl` | 상태 변화, 관측 위치, 경로 길이, 비용, 선택 이유 |
| `result.json`, `result.csv` | 결과와 비용·시간 요약; baseline은 `baseline.json` |
| `trajectory.csv` | 시간별 pose·상태·주행 허용 여부 |
| `runtime.json` | 실제 노드·토픽·타입·서비스 확인 자료 |
| `supervisor.json` | runner 인수와 미션 종료 코드 |
| `screen.mp4`, `record.log` | `--record` 지정 시 녹화 결과와 ffmpeg 로그 |

`selected_execution_sim_seconds`와 `selected_execution_wall_seconds`는 **선택 확정부터
목적지 도착까지**를 각각 시뮬레이션 시간과 실제 벽시계 시간으로 기록한다.
실패 실행은 실패 시점까지의 경과이며 `success`/`execution_completed`를 함께 확인한다.
`approach_until_detection_*`, `stop_*`, `planning_*`, `detection_to_decision_*`는
접근·감지까지, 실제 정지까지, 경로 계산, 판단 확정까지의 시간을 구분한다.
`total_*`는 준비 완료 후 전체 미션 경과이며 `startup_wall_seconds`는 초기 준비,
`process_wall_seconds`는 준비를 포함한 프로세스 경과다. 실행하지 않은 후보의 실측 시간은 만들지 않는다.

초기 A 완주에서 실제 기록된 계산 예시는 다음과 같다. 이는 파라미터에 고정한 답이
아니며 정지 pose·관측·계획 결과에 따라 달라진다.

```text
L_clear=6.3461 m, L_detour=15.5288 m
T_remove=53.7305 s, T_detour=77.6441 s
choice=REMOVE, reason="T_remove + margin < T_detour"
```

이 예시 실행 폴더 이름은 `20261116_174631_lidar_stop`이지만 실제 설정과 결과는
`stage=full`이다. 이름만으로 단계나 검증 범위를 해석하지 않는다.
장애물 없는 기본 주행은 `20261116_174241_baseline_active/baseline.json`에
두 Nav2 목표 성공, 최종 위치 약 `(5.912, -0.037)`, `/scan` 197개,
odom 이동거리 약 7.815 m가 기록되어 있다. 나머지 최종 결과는 검증 기록을 기준으로 한다.

`rviz/namo.rviz`는 정적 지도, 로봇, 실제 LiDAR, local/global costmap,
파란 **HYPOTHETICAL after MOCK REMOVE** 경로, 주황 **DETOUR** 경로,
관측 물체·판단 위치·예상 시간·선택·현재 상태를 표시한다.
파란색은 제거를 가정한 계산 경로이며 아직 실제로 통과 가능한 경로라는 뜻이 아니다.

## 1~2분 발표 영상 준비

먼저 화면을 확인한 뒤 A/B를 각각 녹화한다. 아래 명령은 녹화 기능을 실행하는 방법이며,
현재 장치에서 GUI와 녹화가 검증되었는지는 `docs/VALIDATION.md`를 확인한다.

```bash
python3 namo_sim/scripts/run_demo.py --scenario A --gui --rviz --record --label video_A
python3 namo_sim/scripts/run_demo.py --scenario B --gui --rviz --record --label video_B
```

녹화기는 이번 실행의 Gazebo·RViz 창만 찾아 옆으로 배치하고 창 가림 여부를 확인한다.
두 창을 녹화 중 위에 유지하며, 기본 영상은 **1280×720, 20 fps**다. 데스크톱 독·패널을
제외한 실제 좌표는 `window_layout.log`에 남는다. 창의 최소 크기가 더 크면 화면 안에서
캡처 폭을 조정한다. RViz의 설정 패널은 기본 숨김이며 지도와 상태를 넓게 보여 준다.
배치를 확인할 수 없으면 녹화를 시작하지 않는다. `--record` 없이 수동 화면 배치도 가능하다.

권장 편집 순서는 A의 접근·감지·정지 → 파란/주황 후보와 비용 →
`MOCK REMOVE` 대기·적치·새 스캔 확인 → 짧은 통로 도착 →
B의 같은 판단식과 DETOUR 선택 → 물체가 남아 있는 우회·도착이다.
원본 실행이 2분을 넘으면 초기 준비·반복 주행을 줄여 1~2분으로 편집하고,
배속 구간에는 배속을 표시한다. 조작 대기 시간을 영상 편집으로 줄여 보여도
로그의 실제 22초 대기 및 실측 결과와 구분해 설명한다.
화면 녹화에 실패하면 `record.log`를 확인하고 동일한 두 명령에서 `--record`를 빼서
실행한 뒤 사용 가능한 데스크톱 녹화기로 같은 화면을 기록한다.

전체 결과 CSV와 실제 궤적 그림은 다음 명령으로 다시 만든다.

```bash
python3 namo_sim/scripts/summarize_runs.py
```

제거 실패 검증 명령은 아래와 같다. 정상 A/B와 구분한 오류 주입 실행이며 두 비용/실행
설정의 대기 시간을 함께 1초로 줄여 실패를 빠르게 확인한다. 실패 종료 코드 1이 예상 결과다.

```bash
python3 namo_sim/scripts/run_demo.py --scenario A --remove-model missing_test_object --mock-seconds 1 --label removal_failure
```

## 파일 역할과 실물 전환

| 파일/디렉터리 | 역할 |
| --- | --- |
| `launch/sim.launch.py` | 시뮬레이션·위치 추정·Nav2 전용 launch |
| `config/nav2.yaml`, `config/navigate*.xml` | costmap·planner·controller·복구 없는 주행 BT |
| `config/mission.yaml` | 공통 비용, 감지, timeout, 모의 제거 설정 |
| `tools/generate_worlds.py`, `maps/`, `worlds/` | 일치하는 정적 지도와 실제 벽 |
| `tools/generate_robot.py`, `models/` | 재사용 로봇 SDF와 LiDAR용 장애물 대체 모델 |
| `namo_sim/baseline.py` | 장애물 없는 S→P→G 주행 검증 |
| `namo_sim/planning.py` | 정적 A*, 경로 길이, 비용 선택 규칙 |
| `namo_sim/perception.py` | 지도 차분 클러스터, 연속 관측, 제거 후 광선 확인 |
| `namo_sim/scan_clearing.py` | 실제 인접 광선으로부터 보수적인 clearing 전용 입력 생성 |
| `namo_sim/mission.py` | TF·감지·취소·정지·계획·선택·실행·기록 |
| `namo_sim/mock_remover.py` | 독립된 모의 제거 요청·완료·실패 실행기 |
| `namo_sim/velocity_gate.py` | 단일 베이스 명령 발행과 정지/heartbeat 감시 |
| `rviz/namo.rviz` | 중간 점검 화면 |
| `scripts/run_demo.py`, `scripts/inspect_runtime.py` | 실행·초기화·정리·런타임 인터페이스 기록 |
| `test/`, `docs/` | 핵심 검증과 환경·실행 증거 |

실물에서는 Gazebo·모델 이동 실행기를 실제 LDS02/odom/TF 및 인식·파지·적치 실행기로
교체한다. 제거 요청 `/namo/remove` (`std_srvs/Trigger`)는 수락 여부를 반환하고,
`/namo/removal_status` (`std_msgs/String` JSON)는 대기·실행·완료·실패를 전달한다.
실제 팔 어댑터는 이 경계를 유지하면서 기존 `/find_object`, MoveIt,
팔 trajectory/gripper 인터페이스를 연결할 수 있다.

OpenMANIPULATOR를 포함한 실제 footprint·회전 공간, 카메라/팔 보정,
파지 반복 성공률, 실제 조작 시간, 장애물 종류·이동 가능성 판정은 추가 검증 대상이다.
기존 control과 joystick은 둘 다 `/cmd_vel`을 직접 발행하므로 실물 연결 전에
명령 중재·비상 정지 정책을 통합한다. 현재 시뮬레이션은 설치된 팔 시각 모델을
추가하지 않았으며 베이스와 LiDAR 주행에 집중한다.

A/B를 한 번씩 완주 녹화하고 무배속 영상으로 합치는 명령:

```bash
python3 namo_sim/scripts/record_ab.py
```

두 실행이 모두 성공해야 `.namo/demo_ab.mp4`와 `video_manifest.json`을 만든다.
최종 판정·CSV·실제 궤적은 `.namo/results.csv`, `.namo/validation_summary.json`,
`.namo/route_comparison.png` 및 각 run 폴더에서 함께 확인한다.
