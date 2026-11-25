# 기존 환경 조사 기록

조사일: 2026-11-16 (Asia/Seoul). 작업 경로는
`/home/user2/mobility_simulation/namo_ws/src`이다. 이 기록은 설치 파일과 기존 소스에서
확인한 사실이며, 시뮬레이션 완주 결과는 실행 검증 기록과 구분한다.

## 보존 범위

기존 패키지는 `control_pkg`, `vision_pkg`, `joystick_pkg`, `eated_interfaces`이다.
이 네 패키지에는 NAMO, 지도, Nav2 launch, `/scan` 장애물 판별 구현이 없다.
작업 디렉터리와 상위 경로에서 적용할 `AGENTS.md`는 발견되지 않았고, 기존 네
패키지에 README도 없다. `.git`과 `.agents`는 이 실행 환경에서 빈 읽기 전용
디렉터리로 보이며 `git status --short`는 저장소가 아니라는 오류를 반환했다.
따라서 Git 변경분 비교를 할 수 있다고 주장하지 않는다.

기존 소스 53개 파일의 작업 시작 시 SHA-256은
[original_sha256.json](original_sha256.json)에 보관했다. 원본은
`/tmp/namo_original_hashes.json`이다. 새 구현은 `namo_sim`에 추가하며 기존
하드웨어 launch/config를 수정하지 않는다.

## 설치 환경

| 항목 | 실제 설치 값 |
| --- | --- |
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| ROS | ROS 2 Humble, `/opt/ros/humble` |
| 기본 오버레이 | `/home/user2/turtlebot3_ws/install` |
| Gazebo Classic | `gazebo 11.10.2+dfsg-1` |
| gazebo_ros | `3.9.0-1jammy.20260907.222900` |
| gazebo_msgs | `3.9.0-1jammy.20260907.213014` |
| Nav2 bringup | `1.1.20-1jammy.20260908.021125` |
| Nav2 messages | `1.1.20-1jammy.20260907.223327` |
| TurtleBot3 Gazebo | `2.3.8-1jammy.20260908.002354` |
| ros_gz_sim (동시 설치됨) | `0.244.26-1jammy.20260907.235058` |
| 화면 환경 변수 | `DISPLAY=:0` 설정됨; 이것만으로 GUI 실행 성공을 의미하지 않음 |

Gazebo Sim용 `ros_gz`도 설치되어 있으나, 새 시연에서는 이미 설치된
Gazebo Classic + `gazebo_ros` 조합을 사용한다. ROS/Gazebo 버전 이관은 하지 않는다.
위 버전은 `dpkg-query -W`와 `/etc/os-release`에서 확인했다.

## 기존 코드의 인터페이스

| 인터페이스 | 형식 | 기존 사용 주체 / 의미 |
| --- | --- | --- |
| `/find_object` | `eated_interfaces/action/FindObject` | vision 서버, control 클라이언트 |
| `/realsense/d435i/color/image_raw` | `sensor_msgs/msg/Image` | vision RGB 입력 |
| `/realsense/d435i/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | vision 정렬 depth 입력 |
| `/realsense/d435i/color/camera_info` | `sensor_msgs/msg/CameraInfo` | vision 내부 파라미터 입력 |
| `/voice/robot_command` | `eated_interfaces/msg/RobotCommand` | control에 fetch/stop 명령 |
| `/control/command_status` | `eated_interfaces/msg/RobotCommandStatus` | 명령 수락 결과 |
| `/control/state` | `std_msgs/msg/String` | 파지·운반 상태 |
| `/control/target_class` | `std_msgs/msg/String` | 선택한 인식 대상 |
| `/control/manual_drive_enabled` | `std_msgs/msg/Bool` | control이 발행, joystick이 구독; transient local |
| `/control/release_countdown_sec` | `std_msgs/msg/Float32` | 손바닥 확인 후 놓기 카운트다운 |
| `/vision/open_palm` | `std_msgs/msg/Bool` | 손바닥 감지 |
| `/joy` | `sensor_msgs/msg/Joy` | control과 joystick이 구독 |
| `/joint_states` | `sensor_msgs/msg/JointState` | control 입력 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | control과 joystick이 모두 직접 발행 |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 팔 관절 이동 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 팔 홀드 명령 |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 그리퍼 |
| `/move_action` | `moveit_msgs/action/MoveGroup` | 파지 자세 계획 |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | 수직 후퇴 경로 |
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | MoveIt 계획 실행 |

이 표는 소스에 선언된 인터페이스다. 기존 하드웨어 노드를 실행하여 확인한 실시간
발행자 목록은 아니다. 새 시뮬레이션의 실제 `/scan`, `/odom`, `/tf`, `/cmd_vel`
발행자는 별도 실행 검증에서 확인해야 한다.

`FindObject` 목표는 `target_class`, 결과는 `success`, XYZ, confidence, frame_id,
message이다. vision은 RGB/depth를 동기화하고 TF로 `base_link`에 위치를 변환한다.
YOLO `model_path`는 빈 값일 때 오류가 나며, 이 소스 트리에 가중치 파일은 없다.
중간 점검 시뮬레이션은 vision이나 실제 카메라를 실행할 필요가 없다.

기존 control은 인식 → 파지 → 적재 → 수동 주행 → 손바닥 확인 후 놓기를 다룬다.
Nav2 주행 미션과 동일하지 않으므로 새 미션 관리자를 분리한다. 기존 코드의
작업 식별자 및 지연된 액션 취소 처리 방식은 구현 시 참고할 수 있다.

## 베이스 명령 중재와 비상 정지

기존 `twist_mux`는 없다. control과 joystick이 직접 같은 `/cmd_vel`을 발행한다.
control은 수동 주행 불허 상태에서 반복하여 0 속도를 발행하므로 새 Nav2와 함께
실행하면 명령이 충돌할 수 있다. 시뮬레이션 launch는 기존 control/joystick을
자동으로 시작하지 않고 전용 속도 게이트를 둔다.

기존 비상 정지는 팔 액션 취소, 베이스 정지, 팔 홀드 후 `ERROR`로 전환하고
**조이스틱 권한을 다시 허용**한다. 이것을 NAMO 판단·모의 제거 중 Nav2 명령을
차단하는 정지 잠금으로 간주하면 안 된다. 실제 하드웨어 연결 시 중재 및 정지
정책을 통합해야 한다.

## 설치된 TurtleBot / MoveIt 재사용 가능 범위

`/home/user2/turtlebot3_ws/src`에는 Waffle Pi URDF와 manipulation 모델,
MoveIt 설정, Nav2 설정이 있다. 설치된 manipulation 컨트롤러 관절은
`joint1`~`joint4`, 그리퍼는 `gripper_left_joint`이다.

`turtlebot3_manipulation_description/gazebo/turtlebot3_waffle_pi.gazebo.xacro`는
`libgazebo_ros_diff_drive.so`를 사용한다. wheel separation은 0.287 m, diameter는
0.066 m이며 odometry frame은 `odom`, robot base는 `base_footprint`이다.

설치된 manipulation Nav2 YAML에는 DWB controller, Navfn planner, robot radius
0.22 m, `/scan` 기반 local/global obstacle layer가 있다. 설정의 참고로 쓸 수
있지만 새 시연의 통로·회전 공간에 맞게 footprint/여유를 다시 확인해야 한다.

팔 시뮬레이션 launch는 즉시 재사용할 수 있는 상태가 아니다.
`turtlebot3_manipulation_moveit_config/launch/moveit_gazebo.launch.py`가
`turtlebot3_manipulation_bringup/launch/gazebo.launch.py`를 include하지만,
실제 설치된 bringup launch 디렉터리에는 `base.launch.py`, `hardware.launch.py`만
존재한다. 기본 주행을 우선하고 팔 모델 통합은 후속 작업으로 둔다.

## 설치된 Nav2 액션 필드

`/opt/ros/humble/share/nav2_msgs/action`의 실제 인터페이스를 읽었다.

- `ComputePathToPose`: goal, start, planner_id, use_start → path, planning_time.
  이 설치본 결과에는 `error_code`가 없다. ROS action 결과 상태, 경로 비어 있음,
  서버 준비/응답 timeout을 구분해야 한다.
- `NavigateToPose`: pose, behavior_tree → `std_msgs/Empty result`.
- `FollowPath`: path, controller_id, goal_checker_id → `std_msgs/Empty result`.

후보 경로의 동일 출발점은 정지 확인 후 얻은 현재 pose이며,
`ComputePathToPose.use_start=true`로 명시할 수 있다.

## Gazebo 모의 제거 서비스 계약과 정적 모델 주의점

설치 헤더 `/opt/ros/humble/include/gazebo_ros/gazebo_ros_state.hpp`에 따르면
`libgazebo_ros_state.so`는 world plugin이며 `get_entity_state`와
`set_entity_state` 서비스를 제공한다. 네임스페이스는 world의 plugin 설정에
따른다. factory plugin만 띄웠다고 이 서비스가 자동 제공된다고 가정하지 않는다.

설치된 `gazebo_msgs/srv/SetEntityState.srv` 계약:

```text
gazebo_msgs/EntityState state
---
bool success
```

`EntityState`에는 name, pose, twist, reference_frame이 있으며 reference_frame이
비어 있거나 `world`이면 관성 world 좌표다. 요청에서 quaternion `w=1.0`과
0 twist 등을 명시한다. 응답에 `status_message` 필드는 없다.
설치 라이브러리 심볼에는 `GazeboRosStatePrivate::SetEntityState`와
`gazebo::physics::Entity::SetWorldPose` 호출이 확인된다.

설치 예제 `/opt/ros/humble/share/gazebo_ros/worlds/gazebo_ros_state_demo.world`
17행은 다음 제약을 명시한다:

> When setting state, note that the changes for static entities may not be visible on gzclient

따라서 정적 모델을 이동시켰을 때 물리/센서 상태와 화면 표시가 일치한다고
미리 단정하지 않는다. 제거 대상은 적절한 inertia/gravity를 가진 동적 모델로
구성하는 방법을 우선 검토하고, 실제 서비스 성공·새 `/scan` 관측·costmap 갱신·
Gazebo 화면 표시를 각각 확인한다. 여기서는 헤더와 설치 예제를 조사했으며
정적 모델 이동의 런타임 성공을 검증하지 않았다.

시뮬레이터 모델 이름은 모의 제거 실행기의 조작 핸들에만 사용한다.
감지와 비용 계산에 모델의 실제 pose를 공급하지 않는다.

## 신규 구현과 실물 전환 경계

새 `namo_sim`에서 전용 world/map, launch/config, 초기 위치 설정, LiDAR/TF 감지,
정지 게이트, 정적 지도 A*, Nav2 우회 후보, 비용 판단, 모의 제거, RViz 표시,
CSV 기록을 담당한다. 기존 하드웨어 패키지는 보존한다.

최종 실물 전환 시에는 모의 제거 요청/완료/실패 인터페이스 뒤의 실행기를
기존 vision 및 MoveIt 기반 인식·파지·적치 어댑터로 교체한다. 실제 LDS02/odom/TF,
모바일 매니퓰레이터 footprint, 베이스 중재, 비상 정지, 조작 시간과 성공률,
카메라 보정 및 파지 실패 처리는 추가 검증 대상이다. LiDAR만으로 이동 가능
물체의 종류나 파지 성공을 판정했다고 표현하지 않는다.
