> 이전 demo_ab.mp4의 전방 이동 마운트 모델 기록입니다. 현재 모델은 MODEL_PHYSICS.md를 참조하세요.

# 물리 모델 가정과 좌표

이 패키지는 설치된 ROBOTIS Waffle Pi + OpenMANIPULATOR-X URDF와 RealSense D435i 매크로의 **별도 생성 복사본**을 사용한다. 기존 EATED 소스와 설치된 로봇 파일은 수정하지 않는다. `tools/generate_robot.py`가 `models/robot.urdf`를 생성한다.

바퀴의 유일한 구동자는 Gazebo diff-drive plugin이다. ros2_control에서는 바퀴 명령/상태 인터페이스를 제거했고 별도 joint-state plugin은 읽기만 한다. 기본 `wheel_acceleration:=0.0`은 diff-drive 내부의 누적 속도 제한기를 끄며, 유한 토크 `fmax`를 갖는 물리 모터 속도 제약은 유지한다. 허용된 주행 명령은 gate에서 선형 0.15 m/s², 각속도 0.5 rad/s²로 증가를 제한한다. 안전 조건이 거짓이면 완화 과정 없이 즉시 0을 발행한다. 이 변경은 차체/바퀴 pose를 직접 설정하지 않는다.

두 바퀴의 `fdir1`은 설치된 일반 TurtleBot3 Waffle Pi SDF와 같은 `0 0 0`으로 설정하여 접촉 법선에서 유효한 마찰 접선 두 개를 자동 생성하게 한다. Combined URDF의 원래 값 `1 0 0`은 회전하는 바퀴 collision 좌표계에 고정되어 있었다. [Gazebo 구현](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEPhysics.cc)은 이 방향을 월드 좌표계로 회전하며, [접촉 마찰 설명](https://classic.gazebosim.org/tutorials?cat=physics&tut=physics_params)처럼 접촉 법선과 교차해 다른 마찰 방향을 얻는다. 수정 전 정지 명령 상태에서 실제 바퀴 각도가 약 π/2일 때 이 방향과 지면 법선의 내적 절댓값이 좌 0.99944, 우 0.99846으로 측정되어 접선 기저가 퇴화했다. 이 변경은 바퀴의 방향 값만 수정하며 마찰계수·접촉 강성·중력·모터 토크·world solver는 유지한다. 미끄러짐 개선 여부는 재기동 후 동일 조건의 물리 검증 결과로 판단한다. 기존 기록은 `src/.namo_manip/model_check/creep_diagnostic.json`에 보존한다.

## 차체 질량과 안정성

[ROBOTIS 공식 사양](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)의 Waffle Pi 질량은 SBC·배터리·센서를 포함해 1.8 kg이다. 이 수치에 맞춰 별도 모델링한 두 바퀴(각 0.02849894 kg), 두 캐스터(각 0.005 kg), LDS(0.114 kg)를 제외한 `base_link` 질량을 **1.61900212 kg**으로 둔다. 차체 무게중심 `(-0.064, 0, 0.047) m`는 차체 충돌 상자의 중심이며, 관성은 0.266 × 0.266 × 0.094 m 균일 상자로 계산한다. **무게중심·질량 분배·상자 관성은 실측값이 아닌 시뮬레이션 가정**이다.

기본 모델에는 **0.25 kg 후방 추**를 추가한다. 추는 노란색 0.06 × 0.14 × 0.04 m 상자로 표시되며 `base_link` 기준 `(-0.16, 0, 0.13) m`에 고정 장착한다. 차체 위에 있는 실제 충돌·질량 요소이며 바닥 지지대나 숨긴 고정 장치가 아니다. `ros2 launch namo_manipulation model.launch.py rear_ballast_mass:=0`으로 제거하거나 질량을 변경할 수 있다. 생성 단계에서도 `python3 tools/generate_robot.py --rear-ballast-mass 0.25`를 지정할 수 있다.

원본 복사본의 차체 관성 원점은 바퀴 축 x=0에 있었고, 질량 합은 2.23796 kg이었다. URDF 전 링크의 FK와 관성 원점으로 계산한 전체 무게중심 x는 다음과 같다. 앞 지지 경계는 구동 바퀴 축 x≈0이다.

| 자세 | 수정 전 x (m) | 수정 후 x (m) |
|---|---:|---:|
| STOW | +0.00026 | -0.05231 |
| PREPARED | +0.01142 | -0.04318 |
| PICK: EE=(0.30,0,0.0675), pitch=45° | +0.04325 | -0.01712 |
| PLACE: EE=(0.12,0.33,0.0675), pitch=45° | +0.01950 | -0.03657 |

수정 후 전체 질량은 2.73406 kg이다. 이 계산은 정적 지지 가능성을 점검하며 동적 파지 성공을 보증하지 않는다. 실제 하드웨어 적용 전 차체·배터리·팔·카메라·후방 추의 질량과 무게중심, 허용 하중 및 장착 강도를 다시 측정해야 한다. 중력 해제, world 고정 관절, 객체 순간이동, 부착 플러그인은 사용하지 않는다.

2026-11-17 독립 Gazebo 검증에서 수정 후 PREPARE 동작 중 pitch는 0.1811–0.1825°, PREPARED 유지 중 0.1825–0.18334°였고 roll의 절댓값은 0.0008° 미만이었다. 읽기 전용 차체 자세 기록은 `src/.namo_manip/model_check/stability_after.json`, 정지 자세 비교는 `prepared_pose_after.json`에 있다. 같은 모델의 영상 시각 TF를 사용한 ground projection은 odom 기준 z=0.00063–0.00076 m였다(`src/.namo_manip/perception_odom_check`). 이 검증은 차체 안정화와 카메라 변환 정합에 대한 결과이며, 파지 성공 여부는 별도 실험 결과를 따른다.

## 카메라와 레이저

기존 vision 설정의 `end_effector_link → d435i_link`는 xyz=(-0.065, 0.0195, 0.052) m, rpy=(0,0,0)이다. 이는 기존 설정을 옮긴 값이며 실측 hand-eye 보정 결과라고 가정하지 않는다. 공식 RealSense 매크로의 screw-frame 내부 오프셋을 상쇄하여 이 변환을 그대로 유지한다. D435i 충돌 상자는 0.02505 × 0.09 × 0.025 m이며 EE 기준 중심은 (-0.065, 0.002, 0.052) m이다.

RGB와 depth는 같은 Gazebo depth sensor에서 같은 color-camera 원점·해상도·시각으로 생성한다. Gazebo 카메라의 +X 광축과 optical frame의 +Z는 기존 optical 회전 rpy=(-π/2,0,-π/2)로 일치한다. RGB는 640×480 rgb8, depth는 32FC1(m), 둘의 frame은 `d435i_color_optical_frame`이다. `base_footprint`는 실제 차체와 함께 기울 수 있으므로 바닥 z=0 확인은 영상 시각의 optical→odom TF로 수행해야 한다. 객체의 Gazebo 정답 위치를 인식 좌표 보정에 사용하지 않는다.

LDS 원점은 base_link 기준 (-0.064,0,0.121) m, 평평한 차체의 base_footprint 기준 높이 0.131 m이다. 기본 스캔은 360개 광선, 10 Hz, 범위 0.12–8 m이다. 직경 0.05 m·높이 0.135 m 캔은 레이저 높이와 4 mm만 겹치므로 차체 기울기나 지면 오차에 민감하다.

## 접촉과 캔

팔의 힘 PID는 Kp=80, Kd=0.10, Ki=0이다. 초기 Kd=1.0 실험에서는 정지 목표에서도 관절 4 속도 약 1.27 rad/s가 번갈아 나타나는 수치 진동과 차체 이동이 기록되었다. Kd=0.10 재검증에서는 같은 단계의 정지 관절 속도가 약 0.001 rad/s 이하로 줄었다. 제어는 실제 물리 관절에 힘을 가하며 위치를 매 주기 덮어쓰지 않는다. 초기 관절 자세는 한계에서 여유가 있는 (0, -1.0, 0.8, 0.2) rad이며, 미션이 STOW를 명령하고 확인한 뒤 주행한다.

복사 원본 URDF의 팔 토크 제한 1 N·m에서는 손목 카메라를 편 자세에서 어깨 관절이 포화되어 약 0.16 rad의 위치 오차가 남고 운반 궤적이 실패했다. 별도 시뮬레이션 모델의 팔 4관절에는 **2 N·m의 유한 토크 한계**를 적용한다. 이는 시뮬레이션 설정값이며 실물 연속 허용 토크의 측정값이 아니다. [ROBOTIS XM430-W350 사양](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)의 12 V 정지 토크는 4.1 N·m이고, 이 수치 또한 연속 운전 정격으로 사용하지 않는다. 그리퍼의 1 N 제한은 유지한다.

팔과 양쪽 그리퍼는 Gazebo `position_pid` 힘 제어를 사용한다. 양쪽 손가락은 개별 controller가 같은 목표를 받고 실제 각 관절 상태를 보고한다. 접촉 패드 상자는 0.05 × 0.008 × 0.025 m, 각 질량 0.01 kg, 마찰계수 0.8, 최대 닫힘 힘 1 N이다. 이 값은 접촉 검증을 위한 가정이다.

힘 제어 주기는 500 Hz로 유지한다. JointStateBroadcaster의 지원되는 `use_local_topics=true` 설정으로 원래 관절 상태를 `/joint_state_broadcaster/joint_states`에 발행하고, 설치된 C++ `topic_tools throttle`이 외부 `/joint_states` 전달만 최대 50 Hz(wall clock)로 제한한다. 각 메시지의 simulation stamp와 실제 관절값은 바꾸지 않는다. 바퀴의 별도 joint-state publisher는 기존 30 Hz를 유지한다. 이것은 각 Python 구독자가 불필요하게 500 Hz 메시지를 모두 처리하는 부하를 줄이기 위한 설정이다.

30초간 실제 수신 빈도는 local arm 498.49 Hz, 외부 arm 45.27 Hz, 외부 wheel 29.23 Hz였다(`creep_diagnostic.json`). 손가락의 기존 `fdir1`은 없으므로 자동 접선 설정이며, 바퀴 보완에서 변경하지 않았다.

캔은 직경 0.05 m, 높이 0.135 m의 원통이며 **빈 캔 질량은 실측값이 없어 0.015 kg으로 가정**한다. 중력과 충돌이 활성화되고 얇은 원통의 관성을 사용한다. RGB 표현을 변경해도 이 충돌 형상과 질량은 유지한다. 실제 파지·운반·놓기는 접촉 마찰로 검증하며 결과와 실패는 별도 실행 보고서에 기록한다.

캔 옆면은 `tools/generate_can_label.py`로 만든 SVG/PNG와 COLLADA 시각 메시를 사용한다. 이 자료는 코드로 그린 음료 캔 라벨의 근사 표현이며 실물 사진·공식 라벨 자료가 아니다. 인식 노드에 정답 클래스·위치·신뢰도를 주입하지 않으며, 이 표면을 렌더링한 실제 카메라 영상으로 YOLO의 검출 여부를 다시 평가한다. 단색 빨간 캔에서 실제 YOLO가 검출하지 못했던 기존 증거도 보존한다.

## 재기동 경계

생성 URDF 변경은 다음 Gazebo 기동부터 적용된다. 이전 독립 검증 프로세스(domain 43)는 초기 720개 광선 모델을 사용했으나 안정성 보완 후 재기동한 모델은 360개 광선이다. 센서·관절·시뮬레이션 시계가 함께 초기화되도록 전체 실행을 재시작한다. 기존 실행에 새 TF만 덮어쓰지 않는다.

## 반복 실행의 초기 자세

시작 관절값은 `[0, -0.8, -0.5, 1.3] rad`다. 이전 `[0,-1,0.8,0.2]`는 제어기가 활성화되기 전 중력으로 접히는 동안 joint3가 상한에 가까워져 충돌 검사에서 시작 자체를 거부하는 경우가 있었다. 새 값은 여유 있는 초기 배치이며, 이후에는 실제 관절 제어로 STOW에 이동하고 측정값을 확인해야 주행한다. 작업 중 객체·링크 위치를 보정하는 방식이 아니다.

팔 계획 서비스와 실행 타이머는 같은 상호 배제 callback group을 사용한다. 센서 구독은 별도 group에서 받아 충돌/IK 계산 중에도 주행 잠금 heartbeat, 관절, odometry, 접촉 관측을 갱신한다. 기존 0.75초 신선도 제한과 베이스 이동 제한은 유지한다.
