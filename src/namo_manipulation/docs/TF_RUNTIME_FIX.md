# 이번 시뮬레이션에만 적용하는 TF 교착 수정

현재 시스템의 `ros-humble-tf2`와 `ros-humble-tf2-ros`는 0.25.23이다. 이 버전의 `waitForTransform`과 `testTransformableRequests` 사이 교착은 [공식 geometry2 #992](https://github.com/ros2/geometry2/issues/992)에 보고되어 있고, [0.25.24 변경 기록](https://github.com/ros2/geometry2/blob/0.25.24/tf2_ros/CHANGELOG.rst)은 해당 수정의 backport를 명시한다.

실제 실행에서 `/odom`과 외부 TF는 계속 갱신되지만 Nav2의 `received_global_plan` 시각과 변환 좌표가 고정되고 같은 속도 명령이 새 메시지로 반복됐다. 디버거 스택에서 TF 수신 스레드는 `testTransformableRequests` callback의 mutex에, LaserScan 스레드는 `waitForTransform → addTransformableRequest`의 다른 mutex에 대기했다. 증거: `src/.namo_manip/runs/20261117_023608_nav_tf_stack/navigation.log`(Thread 15/16), 해당 실행의 `navigation_probe.jsonl`. 따라서 속도 명령 수신 시각만 감시해서는 이 오류를 잡을 수 없다. 새 gate는 변환 경로의 메시지 시각과 수신 시각을 함께 검사한다.

## 준비와 적용 범위

```bash
cd /home/user2/mobility_simulation/namo_ws/src
python3 namo_manipulation/tools/prepare_tf2_runtime.py
```

이 명령은 공식 ROS apt 저장소의 두 `.deb`(합계 약 411 kB)를 내려받고 SHA256을 확인한 뒤 `.namo_manip/runtime_fix/root`에 **추출만** 한다. `apt install`, 시스템 패키지 업그레이드, `/opt/ros` 파일 변경은 하지 않는다. 기존 하드웨어 실행 환경도 바꾸지 않는다.

`scripts/run_demo.py`는 추출한 라이브러리의 SHA256을 다시 확인하고, 자신의 자식 프로세스에만 `LD_LIBRARY_PATH` 우선 경로를 지정한다. ROS setup을 source한 뒤 적용하므로 시스템 라이브러리에 다시 가려지지 않는다. 버전과 체크섬은 실행 폴더의 `runtime_fix.json`에 저장된다. 터미널 전체 또는 사용자 shell 설정에 export하지 않는다. 직접 ROS launch로 실행하면 이 국소 경로가 자동 적용되지 않으므로 기본 실행에는 runner를 사용한다.

| 패키지 | 추출 버전 | 시스템 설치 버전(유지) |
|---|---|---|
| ros-humble-tf2 | 0.25.24-1jammy.20260915.203746 | 0.25.23-1jammy.20260907.213559 |
| ros-humble-tf2-ros | 0.25.24-1jammy.20260915.204410 | 0.25.23-1jammy.20260907.224752 |

검증한 `.deb` SHA256:

```text
tf2:     f6b5d1d098a9ba9080e2c24cccf98a669f1f8bb878602b5b53bd55a4cfbd0cfc
tf2_ros: cc7ff31644124f5e601576996e078fe2c6d51c6dbf730182d44db4f75651ed08
```

실행 라이브러리 SHA256:

```text
libtf2.so:     8fd2bf11cfa26711f2d03d1a5a897accb147c5b9f415b9c2c796573e1fac1ba7
libtf2_ros.so: 826d779b34748defedb9564e5e8618bc4355cfdc2a249b8b5f9bdea9b54d86f1
```

0.25.23과 0.25.24의 설치 헤더를 재귀 비교했을 때 `tf2_ros/version.h` 외에는 동일했다. 라이브러리 SONAME은 각각 `libtf2.so`, `libtf2_ros.so`로 같고 `ldd`에서 추가 의존성 누락 없이 국소 경로를 선택함을 확인했다. 이 검사는 호환성의 근거이며 실행 성공은 별도 검증 보고서의 실제 결과를 따른다.

2026-11-17 검증: `src/.namo_manip/runs/20261117_024253_baseline_tf24_sensor_debug/result.json`에서 장애물 없는 S→P→G 주행이 `success=true`, `choice=CLEAR`, 종료 코드 0으로 완료됐다. 최종 위치 오차는 0.031301 m, 기록된 총시간은 53.7 simulation seconds / 55.72 wall seconds, 처리한 스캔은 489개였다. 이전 실행의 `/proc` 라이브러리 매핑도 국소 두 파일 로드를 확인했다(`20261117_023935_baseline_tf24/loaded_runtime.json`). 일시적인 TF 수신 지연은 별도 신선도 gate로 감지하고 정지하며, 교착과 구분해 `sensor_diagnostics`와 `navigation_probe.jsonl`에 기록한다.
