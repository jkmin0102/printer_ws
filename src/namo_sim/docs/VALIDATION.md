# 실제 환경 검증 기록 — 2026-11-16 (KST)

Ubuntu 22.04.5 / ROS 2 Humble / Gazebo Classic 11.10.2 / Nav2 1.1.20에서 실행했다.
ROS 로컬 UDP와 GUI는 실행 제한 밖의 승인된 프로세스로 검증했고, 빌드·로그·영상은
이 작업 공간의 `.namo/` 아래에 격리했다. 기존 실물 노드는 실행하지 않았다.

## 보존과 빌드

- 원본 네 패키지의 53개 파일 SHA256이 모두 동일하다.
- 현재 `.git` 디렉터리는 빈 읽기 전용 디렉터리여서 Git 변경 이력 비교는 불가능했다.
- `control_pkg`, `vision_pkg`, `joystick_pkg`, `eated_interfaces`, `namo_sim` 전체 빌드 성공.
- 기존 제어·비전 로직 28개 테스트 통과. 새 A*/감지/비용/clearing 로직 16개 테스트 통과.
- 자동 생성 lint 테스트는 실행하지 않았다. 하드웨어·YOLO 모델·실물 파지는 검증 범위가 아니다.

실제 사용한 명령:

```bash
cd /home/user2/mobility_simulation/namo_ws/src
colcon --log-base .namo/log build --base-paths namo_sim \
  --build-base .namo/build --install-base .namo/install --symlink-install

colcon --log-base .namo/regression/log build \
  --base-paths control_pkg vision_pkg joystick_pkg eated_interfaces namo_sim \
  --build-base .namo/regression/build --install-base .namo/regression/install --symlink-install

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=namo_sim \
  python3 -m pytest -q namo_sim/test
```

전체 빌드/기존 테스트/해시 기록: `../../.namo/regression/`의 `build.log`,
`original_logic_tests.log`, `original_logic_tests.xml`, `preservation.json`, `summary.json`.

## 순차 실행과 추가 점검

| 실행 | 실제 결과 | 자료 (`.namo/runs/` 아래) |
|---|---|---|
| 장애물 없는 기본 Nav2 | S→P→G 두 액션 성공, 최종 목표 오차 약 9.5cm, 실제 scan197회/odom7.82m | `20261116_174241_baseline_active/baseline.json` |
| LiDAR 감지·정지 | 자유 공간의 실제 점군으로 검출, 목표 취소 후 실제 odom 정지 | `20261116_174631_lidar_stop/events.jsonl` |
| 장애물 x=2.65m로 이동 | 관측 표면 x가 약2.14→2.38m로 변화; 정지 검증 성공 | `20261116_174753_lidar_shifted/result.json` |
| B 판단만 | 같은 비용 파라미터로 DETOUR; 실행 시간은 null | `20261116_174752_B_plan/result.json` |
| B 완주 | 장애물 유지, DETOUR 선택, G 도착; 실제 궤적 최고 y=1.62m | `20261116_174845_B_full/result.json` |
| 감지 켠 장애물 없음 | CLEAR로 G 도착, 벽 때문에 제거 선택하지 않음 | `20261116_174848_clear_with_detector/result.json` |
| 제거 서비스 실패 | 없는 대상 핸들로 Gazebo success=false; FAILED 및 정지, 예상 종료 코드1 | `20261116_174948_removal_failure/result.json` |
| 제거 불가 | A에서도 DETOUR; 판단만 검증 | `20261116_175150_immovable_plan/result.json` |
| 초기화 후 A 반복 | REMOVE→모의 적치→새 scan/costmap 확인→G 도착, 실제 최고 y=-0.016m | `20261116_175538_A_verified_repeat/result.json` |

`lidar_stop`이라는 초기 폴더 이름과 달리 해당 실행의 실제 파라미터는 `stage=full`이다.
초기 CLI override가 노드별 YAML보다 우선하지 않는 문제를 발견한 뒤, runner는 매 실행마다
확정된 `parameters.yaml`을 생성하도록 수정했다. 검증 범위는 폴더 이름이 아니라 결과의
`stage`, `execution_completed` 및 로그를 기준으로 한다.

```bash
python3 namo_sim/scripts/run_demo.py --baseline --label baseline_active
python3 namo_sim/scripts/run_demo.py --scenario A --stage detect --obstacle-x 2.65 --label lidar_shifted
python3 namo_sim/scripts/run_demo.py --scenario B --stage plan --label B_plan
python3 namo_sim/scripts/run_demo.py --scenario B --label B_full
python3 namo_sim/scripts/run_demo.py --scenario A --no-obstacle --label clear_with_detector
python3 namo_sim/scripts/run_demo.py --scenario A --remove-model missing_test_object --mock-seconds 1 --label removal_failure
python3 namo_sim/scripts/run_demo.py --scenario A --immovable --stage plan --label immovable_plan
```

병렬로 실행한 점검은 `--domain 43 --port 11356`, `--domain 44 --port 11357`로 격리했다.
시나리오마다 속도나 제거 시간을 조정하지 않았다. 실패 주입만 명시적으로 양쪽 제거 시간을
1초로 설정했으며 정상 A/B 결과에 섞지 않았다.

## 실행 중 확인한 인터페이스

각 실행의 `runtime.json`에서 실제 ROS 그래프를 확인했다.

| 토픽 | 실제 타입 | 발행 주체 |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo `turtlebot3_laserscan` |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo `turtlebot3_diff_drive` |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 `controller_server` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `velocity_gate` 단독 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map_server` |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 global costmap |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | AMCL, diff drive, robot_state_publisher |

`map→odom→base_footprint→base_link→base_scan` 체인을 사용한다. 센서 시각의 TF로 점군을
변환하고 실제 global costmap에 장애물의 lethal 관측이 생긴 것을 확인한 후 우회 계획을 요청한다.
`NavigateToPose`, `ComputePathToPose`, `/gazebo/set_entity_state`는 설치된 Humble 인터페이스를
읽고 실행했다. Humble 계획 결과에는 `error_code`가 없으므로 실패 status만으로 경로 없음을
단정하지 않고 관측 global grid에서도 불가함을 확인해야 제거 단독 선택을 허용한다.

## 수정 과정에서 드러난 문제

- 기본 Nav2 보조 BT가 실행하지 않는 spin 서버를 요구함: 복구/전체 costmap 초기화 없는 전용 BT로 변경.
- 생성 SDF의 encoding 선언이 spawn_entity 문자열 파서와 충돌: 해당 선언 제거.
- 액션 서버 발견만으로 lifecycle ACTIVE를 가정함: 실제 GetState ACTIVE를 확인한 뒤 목표 전송.
- 최종 반복 중 `CLEARANCE_UNCONFIRMED`로 안전 정지한 실행이 있음
  (`20261116_180450_A_final`). 가정 A*의 모든 셀을 그대로 통과해야 한다는 검사에서,
  관측 대상 영역의 lethal 제거 확인 + 새 LiDAR 통과 광선 + Nav2의 짧은 경로 재계획/충돌 검증으로
  변경했다. 전체 costmap을 지우는 방식은 사용하지 않는다.
- GUI 시작 splash, GNOME 독 영역, 창 포커스 문제를 수정하고 이번 실행의 창 PID만 배치한다.
  `20261116_175648_A_screen`은 화면 확인용 중간 캡처이며 최종 발표 영상으로 사용하지 않는다.
- 추가 관측 저장으로 빈 통로의 `(2.125,-0.125)`, `(2.125,0.175)` 두 셀이 광선 사이에 남는
  현상을 확인했다. `/scan_clearing`을 clearing 전용으로 추가하고, 코너의 부분 관측을
  충분히 확인하도록 A/B 공통 연속 관측을 8회로 강화했다. raw `/scan` 표시·감지는 유지한다.
  `20261116_182155_clearing_visible`에서 새 광선 3회·lethal 소멸·Nav2 짧은 경로 검증 후
  G까지 완주했다. 이 빠른 진단 실행만 대기 시간을 1초로 줄였으며 최종 A/B는 원래22초다.
- `20261116_182239_final_A`는 통로 확인을 통과하여 G에서 약5cm까지 도착했으나
  불필요한 최종 방향 정렬을 반복하다 Nav2 progress timeout으로 실패했다. 과제의 목표는
  위치 도착이므로 XY 허용 오차0.12m를 유지하고 방향은 자유(π rad)로 설정했다.
  이 실패 실행을 성공 완주 영상으로 사용하지 않는다.

## 범위와 남은 확인

실물 파지, D435i/YOLO 실시간 연결, 팔 접촉 물리, 실물 비상 정지·조이스틱 통합은 수행하지 않았다.
`v_eff=0.20m/s`와 전환 시간은 가정값이고 실제 하드웨어에서 보정해야 한다.
실행하지 않은 대안의 실측 시간이나 전역 최적성을 주장하지 않는다.

단일 알려진 이동 가능 장애물이라는 실험 가정이다. 추가 검출 군집은 제거 후보 계산 복사본에도
유지하지만 여러 물체를 제거하는 시퀀스는 구현하지 않는다. 실패 시 자동 재시도 대신 정지하고
이유를 표시한다. 빈 경로·요청 시간 초과·TF 노후화·clock reset 처리는 코드에 구현했으며,
이 모든 오류를 실시간으로 각각 주입해 검증한 것은 아니다.

## 최종 완료 결과와 전달물

최종 정상 A/B의 `parameters.yaml`을 비교해 **scenario 표시값 외 모든 파라미터가 동일함**을
확인했다. 두 실행 모두 실제 Nav2 성공과 최종 TF 위치를 확인했다.

| 시나리오 | 선택 | L_clear | L_detour | T_remove | T_detour | 선택→G 실측 sim / wall | 목표 오차 |
|---|---|---:|---:|---:|---:|---:|---:|
| A | REMOVE | 6.253m | 15.303m | 53.265s | 76.516s | 50.600s / 51.538s | 0.089m |
| B | DETOUR | 6.267m | 9.130m | 53.334s | 45.652s | 36.900s / 37.281s | 0.094m |

- A: [최종 결과 JSON](../../.namo/runs/20261116_182534_final_A/result.json),
  [CSV](../../.namo/runs/20261116_182534_final_A/result.csv),
  [상태 로그](../../.namo/runs/20261116_182534_final_A/events.jsonl).
  제거 후 원본 스캔 3회 연속 통과, 대상 영역 lethal 셀0, 새 Nav2 짧은 경로6.152m를 확인했다.
- B: [최종 결과 JSON](../../.namo/runs/20261116_182651_final_B/result.json),
  [CSV](../../.namo/runs/20261116_182651_final_B/result.csv),
  [상태 로그](../../.namo/runs/20261116_182651_final_B/events.jsonl).
  제거 요청 없이 실제 궤적 최고 y=1.687m를 지나 G에 도착했다.
- [통합 A/B 영상](../../.namo/demo_ab.mp4): **114.2초, 1280×720, 20fps, 약9.4MB**.
  1배속이며 22초 조작 대기를 편집으로 줄이지 않았다. Gazebo의 물체 적치와 B의 물체 유지,
  RViz 비용/선택/경로/완료 화면을 추출 프레임으로 확인했다.
- [영상 실행 목록](../../.namo/video_manifest.json), [전체 CSV](../../.namo/results.csv),
  [실제 궤적 PNG](../../.namo/route_comparison.png), [PDF](../../.namo/route_comparison.pdf),
  [집계 JSON](../../.namo/validation_summary.json).

마지막 추가 회귀에서 정지 후 새 목표 전송 직전 map→odom TF가 잠시 오래되어 Nav2가
목표가 아닌 지점에서 성공을 반환한 경우도 발견했다. 미션의 독립된 최종 위치 검사에서
이를 실패로 잡았고, 목표 전송 전에 TF가 요청 시각 이후0.2초까지 유효한지 확인하도록
보완했다. [마지막 장애물 없는 주행](../../.namo/runs/20261116_182910_clear_fresh_tf/result.json)은
`CLEAR`, 실제 scan197회, 목표 오차 약0.105m로 성공했다. Nav2 성공 status만으로 도착을 주장하지 않는다.

AMCL의 미래 시각 transform 발행 동작은 설치 인터페이스 및
[Humble AMCL 원본 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp)을
함께 확인했다. 센서 데이터는 여전히 각 메시지 시각의 TF로 변환하며 최신 TF로 대체하지 않는다.

최종 영상 생성 명령:

```bash
python3 namo_sim/scripts/record_ab.py
python3 namo_sim/scripts/summarize_runs.py
```

`record_ab.py`는 A/B를 각각 새 프로세스로 초기화하고 둘 다 완주했을 때만 합친 영상을 만든다.
중간 시행착오를 포함한 CSV의 성공/실패 개수는 개발 중 서로 다른 설정의 기록이며,
동일 조건 반복 실험의 성공률로 해석하면 안 된다.
