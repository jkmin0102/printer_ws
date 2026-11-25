from namo_manipulation.drive_safety import drive_permission, slew_command


def evidence(**overrides):
    values=dict(requested=True,lease_age=.05,command_age=.03,arm_age=.02,lock_age=.02,
                arm_stowed=True,arm_locked=False,plan_receipt_age=.02,plan_stamp_age=.05)
    return dict(values,**overrides)


def test_repeated_commands_with_stale_controller_pose_stop_immediately():
    allowed,reason=drive_permission(**evidence(plan_stamp_age=4.))
    assert not allowed
    assert reason=='제어기 위치 정보 갱신 지연'
    assert slew_command((.16,.4),(.15,.35),.05,allowed)==(0.,0.)


def test_fresh_stamp_without_recent_publication_is_rejected():
    assert not drive_permission(**evidence(plan_receipt_age=1.))[0]
    assert not drive_permission(**evidence(plan_stamp_age=-.2))[0]
    assert drive_permission(**evidence(plan_stamp_age=-.09))[0]


def test_any_arm_or_lease_guard_bypasses_acceleration_ramp():
    for change in (dict(arm_locked=True),dict(arm_stowed=False),dict(arm_age=1.),
                   dict(requested=False),dict(lease_age=1.),dict(command_age=1.)):
        allowed,_=drive_permission(**evidence(**change))
        assert not allowed
        assert slew_command((.16,.5),(.16,.5),.05,allowed)==(0.,0.)


def test_resume_ramps_from_zero_and_long_pause_cannot_jump():
    assert drive_permission(**evidence())[0]
    linear,angular=slew_command((.16,.5),(0.,0.),.05,True)
    assert abs(linear-.0075)<1e-10 and abs(angular-.025)<1e-10
    linear,angular=slew_command((.16,.5),(0.,0.),10.,True)
    assert linear<=.015 and angular<=.05
