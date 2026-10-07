import pytest

from nidar_autonomy.arming_guard import ArmRejected, check_arm_preconditions


def test_arm_allowed_when_confirmed_disarmed():
    check_arm_preconditions(False)  # must not raise


def test_arm_rejected_when_already_armed():
    with pytest.raises(ArmRejected):
        check_arm_preconditions(True)


def test_arm_rejected_when_state_unknown():
    with pytest.raises(ArmRejected):
        check_arm_preconditions(None)
