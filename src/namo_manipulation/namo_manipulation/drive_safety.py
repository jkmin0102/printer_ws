"""Pure drive interlock and acceleration policy; no simulator state inputs."""
import math


def drive_permission(*, requested, lease_age, command_age, arm_age, lock_age,
                     arm_stowed, arm_locked, plan_receipt_age, plan_stamp_age,
                     lease_timeout=.75, command_timeout=.5):
    conditions = [
        (requested, '미션 정지 요청'),
        (lease_age < lease_timeout, '미션 허가 만료'),
        (command_age < command_timeout, '주행 명령 수신 지연'),
        (arm_age < .75 and lock_age < .75, '팔 상태 수신 지연'),
        (arm_stowed and not arm_locked, '팔 작업 중 또는 주행 자세 미확인'),
        # Gazebo publishes /clock at 10 Hz and odom TF at 100 Hz. A current
        # transform can therefore lead the latest /clock tick by up to .1 s.
        (plan_receipt_age < .75 and -.15 <= plan_stamp_age < .75, '제어기 위치 정보 갱신 지연'),
    ]
    return all(valid for valid, _ in conditions), next(
        (label for valid, label in conditions if not valid), '주행 조건 확인')


def slew_command(target, previous, dt, allowed, linear_acceleration=.15,
                 angular_acceleration=.5):
    # Stops bypass the ramp. Capping dt prevents a long executor pause from
    # allowing a large restart jump, even when every input is fresh again.
    if not allowed or not all(math.isfinite(v) for v in (*target, *previous, dt)):
        return (0., 0.)
    dt = min(.1, max(0., dt))
    return tuple(old + max(-rate*dt, min(rate*dt, desired-old))
                 for desired, old, rate in zip(target, previous,
                                                (linear_acceleration, angular_acceleration)))
