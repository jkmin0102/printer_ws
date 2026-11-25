# 공식 배치 기반 물리 로봇 모델

현재 `models/robot.urdf`는 ROBOTIS 공식 `turtlebot3_manipulation.urdf.xacro`와 해당 include 파일들을 기준으로 생성한다. 기준 commit은 `814f140e8e208994dd6159f189993156e3943d98`이다.

원본: https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/tree/814f140e8e208994dd6159f189993156e3943d98/turtlebot3_manipulation_description

## 원본과 수정본의 구분

기존 설치 경로 `/home/user2/turtlebot3_ws/src/turtlebot3_manipulation`에는 사용자 수정이 있었다. `git diff HEAD`에서 팔 장착점 x가 -0.092→+0.033m, end_effector_joint가 0.126→0.140m로 바뀌었고, LiDAR와 Pi 카메라 링크들이 삭제되어 있었다. 이전 시연은 이 수정본을 include했으며 LiDAR를 별도로 추가했다.

이번에는 수정된 working tree를 사용하지 않는다. 위 commit의 공식 Xacro 7개를 `models/official/`에 바이트 그대로 보관했다. `manifest.json`에 경로별 SHA256과 commit을 기록하고 생성 때 검증한다. include 경로만 임시 작업 디렉터리에서 연결하고, 원본 파일과 설치된 패키지는 수정하지 않는다. `models/robot_sources.json`은 생성된 URDF 해시, 공식 원본, hand-eye 출처, 시뮬레이션 보완 사항을 기록한다. 메시 자산은 설치된 ROBOTIS/RealSense 패키지를 사용한다.

| 변환 | 공식 원본 유지 값 (m) |
|---|---|
| base_link → link1 (`base_fixed`) | (-0.092, 0, 0.091) |
| base_link → base_scan (`scan_joint`) | (-0.024, 0, 0.122) |
| link5 → end_effector_link | (0.126, 0, 0) |

로봇 전방이 +X다. 따라서 LiDAR가 팔 장착점보다 6.8cm 앞에 있다. 공식 배치 복원 이후 기본 모델에는 노란색 250g 후방 추를 추가하지 않는다. 이전 추·전방 장착 모델의 수치와 영상은 [이전 기록](MODEL_PHYSICS_PREVIOUS_FORWARD_MOUNT.md)에 보존한다.

## EATED hand-eye 정의

기존 `vision_pkg/launch/vision.launch.py` 및 `vision2.launch.py`의 `DeclareLaunchArgument` 기본값을 AST로 읽는다. 두 파일의 값이 다르거나 필요한 항목이 없으면 생성을 중지한다. 원본 vision 파일은 수정하지 않는다.

`end_effector_link → d435i_link`는 **xyz=(-0.065, +0.0195, +0.052)m, rpy=(0,0,0)**이다. 기존 설정값을 사용하는 것이며 실측 hand-eye calibration 결과로 간주하지 않는다.

RealSense 매크로의 origin은 `d435i_link`가 아니라 `d435i_bottom_screw_frame`이다. 생성기는 확장된 RealSense 모델의 screw→link 변환을 읽어 역변환을 합성한다. 현재 screw 장착점은 (-0.0756, 0.002, 0.0395)m이며, 두 관절을 합성한 결과가 EATED 정의와 정확히 일치하는지 검사한다. 카메라 collision box의 중심은 EE 기준 (-0.065, 0.002, 0.052)m이다.

공식 EE는 손목에서 126mm이고, 접촉 패드 중앙의 집기 기준점(TCP)은 140mm이다. 두 좌표계를 혼동하지 않는다. IK는 패드 중앙을 목표로 삼고 카메라 TF는 공식 EE를 사용한다. `robot_geometry.json`은 생성된 URDF에서 어깨 위치, EE 위치 및 카메라 변환을 추출해 실행기의 FK·충돌 검사에 전달한다. 테스트는 실제 URDF 관절 체인을 따라 계산한 FK와 이를 비교한다.

RGB/depth는 같은 Gazebo depth sensor의 color-camera 원점·시각·해상도로 생성한다. RGB 640×480 rgb8, depth 32FC1(m), optical frame은 `d435i_color_optical_frame`이다. Pi 카메라의 공식 TF 링크는 보존하되 사용하지 않는 RGB 센서 플러그인은 제외한다.

## 시뮬레이션 보완과 가정

공식 장착 관절과 메시 배치를 유지하면서 다음 Gazebo용 보완을 별도 생성본에 적용한다. 따라서 공식 URDF를 아무 변경 없이 그대로 실행한다고 주장하지 않는다.

- 바퀴는 Gazebo diff-drive 하나가 제어한다. 미사용 ros2_control 바퀴 핸들을 제거한다. 유한 모터 토크 20Nm를 유지하고 중복 가속 누적기는 끈다. 회전하는 바퀴의 마찰 접선 퇴화를 막기 위해 `fdir1=0 0 0`을 사용한다.
- 차체 질량 1.61900212kg, 무게중심 (-0.064,0,0.047)m와 균일 상자 관성은 시뮬레이션 가정이다. 팔 토크 제한 2Nm, 힘 PID Kp=80/Kd=0.10, 그리퍼 제한 1N도 시뮬레이션 설정이다.
- 패드는 0.05×0.008×0.025m 충돌 상자, 각각 10g, 마찰계수 0.8이다. 양쪽 손가락에 실제 접촉 센서와 개별 힘 PID를 사용한다.
- LiDAR 장착점과 메시·충돌 형상은 공식 원본을 유지하고 센서 플러그인만 LDS-02용 실험 설정(360개 광선, 10Hz, 0.12–8m)으로 교체한다. 이 모델의 평평한 바닥 기준 레이저 높이는 약 13.2cm로 13.5cm 캔과 여유가 작다.
- 캔의 지름 5cm/높이 13.5cm는 사용자 지정이고 빈 캔 질량 15g은 실측값이 없어 가정했다. 중력, 충돌, 양손 접촉을 통해 들고 운반하고 놓는다. 순간이동·부착 플러그인은 없다.
- 초기 팔 관절은 [0,-0.8,-0.5,1.3]rad이며 STOW 확인 후에만 주행한다. 힘 제어는 500Hz, 외부 joint_states 전송은 최대 50Hz다. 오래된 센서·팔 상태에서 주행을 허용하지 않는다.

## 작업 자세와 연속 주행

팔 위치가 공식 위치로 12.5cm 뒤로 돌아왔으므로 이전 정면 집기 좌표를 재사용할 수 없다. 캔의 실제 LiDAR 관측 중심에서 x=0m, y=-0.34m 떨어진 후보를 만들고, 정적 벽·관측 장애물·적치한 캔까지 포함해 반경 0.28m의 베이스 경로를 검사한다. 교차로 접근 뒤 최신 LiDAR와 costmap으로 작업 목표를 다시 산출한다. AMCL 보정으로 달라진 이전 관측 좌표를 그대로 사용하지 않는다. 작업 목표가 점유 영역이면 거부하며 Nav2가 다른 셀로 대체하도록 허용하지 않는다. 최종 작업 자세는 map yaw 0°이며 팔이 측면의 캔을 잡는다. 실제 RGB-D 관측 이후 다시 IK·관절 한계·바닥·차체·LiDAR·벽 충돌을 검사한다. 기본 집기 pitch는 25°, 들어 올리는 거리는 8cm다. 적치 중심은 기존 녹색 적치 공간 안의 map (-0.40,0.90)m다.

적치 후에는 팔 STOW, 연속 LiDAR clearing 및 최신 costmap을 확인한다. **현재 위치에서 G를 유일한 목표로 보낸다.** 별도의 departure waypoint나 그 지점의 정밀 정지·방향 맞추기는 없다. `config/navigate.xml`의 PipelineSequence가 2Hz로 현재 위치에서 경로를 다시 계산하면서 FollowPath를 계속 실행한다. 방향 전환이 필요하면 주행 제어기가 목표 쪽으로 정렬한다. 목적지 이외의 강제 복귀점은 없다. 계산 실패·충돌 예측·센서 신선도 실패 시 정지 동작은 유지한다. 제거 후 또는 우회 중 액션이 중단되면 정지 상태에서 새 LiDAR 4회와 최신 costmap을 받고 G 경로를 검증한 뒤 같은 G 요청을 최대 2회 다시 보낼 수 있다. 정밀 작업 접근·센서 오류에는 이 재시도를 적용하지 않는다.

RViz의 파랑/주황은 판단 시점의 후보 경로이며, **보라색 /plan은 현재 갱신되는 실제 주행 경로**다. `result.json.navigation_goals`에서 각 실제 목표, 시작 위치와 시각을 확인할 수 있다.

## 재생성

```bash
source /opt/ros/humble/setup.bash
source /home/user2/turtlebot3_ws/install/setup.bash
python3 namo_manipulation/tools/generate_robot.py
```

생성은 소스 디렉터리에서 수행한다. 변경된 모델은 전체 Gazebo 재기동 후 적용한다. 기존 실행에 TF만 덮어쓰지 않는다. 실제 실행 성공·실패와 영상은 [검증 기록](VALIDATION.md)을 참조한다.
