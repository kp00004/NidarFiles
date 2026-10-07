from nidar_autonomy.arm_trigger import should_attempt_arm, should_attempt_disarm


def test_arm_on_idle_to_entering():
    assert should_attempt_arm("start", "idle", "entering") is True


def test_no_arm_on_noop_start():
    # start while already past idle is a no-op transition -- must not arm.
    assert should_attempt_arm("start", "entering", "entering") is False
    assert should_attempt_arm("start", "aborted", "aborted") is False


def test_no_arm_on_abort():
    assert should_attempt_arm("abort", "idle", "aborted") is False


def test_disarm_on_abort_from_any_state():
    assert should_attempt_disarm("abort") is True


def test_no_disarm_on_start():
    assert should_attempt_disarm("start") is False
