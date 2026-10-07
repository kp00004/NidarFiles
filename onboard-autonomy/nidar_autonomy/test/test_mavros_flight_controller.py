"""Exercises MAVROSFlightController against a small in-process fake in
place of the real flight_command.FlightCommandClient -- deliberately
does not import rclpy or mavros_msgs, so this suite runs without a ROS
environment, same as the rest of this package's tests."""

from dataclasses import dataclass
from typing import List, Optional

import pytest

from nidar_autonomy.mavros_flight_controller import (
    ArmCommandFailed,
    MAVROSFlightController,
    TelemetryUnavailableError,
)


@dataclass
class FakeArmingResult:
    requested: bool
    success: bool
    state_confirmed: bool
    message: str = "fake result"


class FakeFlightCommandClient:
    """Stands in for flight_command.FlightCommandClient's public surface
    that MAVROSFlightController actually uses."""

    def __init__(self, is_armed: Optional[bool] = None) -> None:
        self.is_armed = is_armed
        self.calls: List[str] = []
        self.next_arm_result: Optional[FakeArmingResult] = None
        self.next_disarm_result: Optional[FakeArmingResult] = None

    def arm(self) -> FakeArmingResult:
        self.calls.append("arm")
        result = self.next_arm_result or FakeArmingResult(
            requested=True, success=True, state_confirmed=True
        )
        if result.success and result.state_confirmed:
            self.is_armed = True
        return result

    def disarm(self) -> FakeArmingResult:
        self.calls.append("disarm")
        result = self.next_disarm_result or FakeArmingResult(
            requested=False, success=True, state_confirmed=True
        )
        if result.success and result.state_confirmed:
            self.is_armed = False
        return result


# -- arm/disarm delegate to the real client, nothing new -------------------


def test_arm_delegates_to_client_and_succeeds():
    client = FakeFlightCommandClient(is_armed=False)
    controller = MAVROSFlightController(client)
    controller.arm()
    assert client.calls == ["arm"]
    assert controller.armed is True


def test_disarm_delegates_to_client_and_succeeds():
    client = FakeFlightCommandClient(is_armed=True)
    controller = MAVROSFlightController(client)
    controller.disarm()
    assert client.calls == ["disarm"]
    assert controller.armed is False


def test_arm_raises_when_client_reports_unconfirmed():
    client = FakeFlightCommandClient(is_armed=False)
    client.next_arm_result = FakeArmingResult(
        requested=True, success=True, state_confirmed=False, message="mismatch"
    )
    controller = MAVROSFlightController(client)
    with pytest.raises(ArmCommandFailed):
        controller.arm()


def test_disarm_raises_when_client_reports_rejected():
    client = FakeFlightCommandClient(is_armed=True)
    client.next_disarm_result = FakeArmingResult(
        requested=False, success=False, state_confirmed=False, message="rejected"
    )
    controller = MAVROSFlightController(client)
    with pytest.raises(ArmCommandFailed):
        controller.disarm()


# -- full (success, state_confirmed) matrix, both directions ---------------
#
# flight_command.ArmingResult can legitimately land on any of the four
# (success, state_confirmed) combinations (see _set_arming's "NOTE:
# /mavros/state nonetheless shows the requested state -- treat with
# caution" branch for how success=False, state_confirmed=True arises for
# real) -- MAVROSFlightController._raise_if_not_confirmed requires BOTH
# to be True, so every other combination, for both arm() and disarm(),
# must raise ArmCommandFailed. Only (True, True) -- already covered by
# test_arm_delegates_to_client_and_succeeds /
# test_disarm_delegates_to_client_and_succeeds -- must not.


def test_arm_raises_when_client_reports_outright_rejected():
    # success=False, state_confirmed=False: the FCU rejected the request
    # and the state agrees it never armed.
    client = FakeFlightCommandClient(is_armed=False)
    client.next_arm_result = FakeArmingResult(
        requested=True, success=False, state_confirmed=False, message="rejected"
    )
    controller = MAVROSFlightController(client)
    with pytest.raises(ArmCommandFailed):
        controller.arm()


def test_disarm_raises_when_client_reports_unconfirmed():
    # success=True, state_confirmed=False: FCU accepted the disarm but
    # /mavros/state never caught up within the confirm timeout.
    client = FakeFlightCommandClient(is_armed=True)
    client.next_disarm_result = FakeArmingResult(
        requested=False, success=True, state_confirmed=False, message="mismatch"
    )
    controller = MAVROSFlightController(client)
    with pytest.raises(ArmCommandFailed):
        controller.disarm()


def test_arm_raises_when_rejected_but_state_confirms_anyway():
    # success=False, state_confirmed=True: the FCU reported rejection but
    # /mavros/state nonetheless shows the requested state (the "NOTE"
    # branch in flight_command._set_arming). The wrapper must still treat
    # this as a failure -- it must not trust the state confirmation over
    # an explicit FCU rejection.
    client = FakeFlightCommandClient(is_armed=False)
    client.next_arm_result = FakeArmingResult(
        requested=True,
        success=False,
        state_confirmed=True,
        message="rejected but state confirms anyway",
    )
    controller = MAVROSFlightController(client)
    with pytest.raises(ArmCommandFailed):
        controller.arm()


def test_disarm_raises_when_rejected_but_state_confirms_anyway():
    # Same success=False/state_confirmed=True mismatch, for disarm().
    client = FakeFlightCommandClient(is_armed=True)
    client.next_disarm_result = FakeArmingResult(
        requested=False,
        success=False,
        state_confirmed=True,
        message="rejected but state confirms anyway",
    )
    controller = MAVROSFlightController(client)
    with pytest.raises(ArmCommandFailed):
        controller.disarm()


def test_arm_precondition_rejection_propagates_unchanged():
    # FlightCommandClient.arm() can raise arming_guard.ArmRejected before
    # ever calling mavros -- that must propagate through this class
    # unchanged, not be swallowed or wrapped.
    class RejectingClient(FakeFlightCommandClient):
        def arm(self):
            raise RuntimeError("stands in for arming_guard.ArmRejected")

    controller = MAVROSFlightController(RejectingClient(is_armed=False))
    with pytest.raises(RuntimeError):
        controller.arm()


# -- read-only state ---------------------------------------------------


def test_armed_raises_when_client_state_unknown():
    client = FakeFlightCommandClient(is_armed=None)
    controller = MAVROSFlightController(client)
    with pytest.raises(TelemetryUnavailableError):
        controller.armed  # noqa: B018


def test_position_raises_telemetry_unavailable():
    controller = MAVROSFlightController(FakeFlightCommandClient())
    with pytest.raises(TelemetryUnavailableError):
        controller.position  # noqa: B018


def test_velocity_raises_telemetry_unavailable():
    controller = MAVROSFlightController(FakeFlightCommandClient())
    with pytest.raises(TelemetryUnavailableError):
        controller.velocity  # noqa: B018


# -- abort and every setpoint-issuing method are out of scope, on purpose --
#
# abort() specifically: INTEGRATION_CHECKPOINTS.md Checkpoint 7 requires
# in-flight ABORT to NOT simply mean "immediately disarm", and what it
# should mean is that checkpoint's own required design pass -- so this
# class must not encode "abort == disarm" as decided-correct behavior
# before that design pass happens.


@pytest.mark.parametrize(
    "call",
    [
        lambda c: c.abort(),
        lambda c: c.takeoff(1.0),
        lambda c: c.land(),
        lambda c: c.hold(),
        lambda c: c.set_position(1.0, 0.0, 0.0),
        lambda c: c.set_velocity(0.5, 0.0, 0.0),
        lambda c: c.set_yaw(1.0),
    ],
)
def test_setpoint_methods_raise_not_implemented(call):
    controller = MAVROSFlightController(FakeFlightCommandClient(is_armed=True))
    with pytest.raises(NotImplementedError):
        call(controller)


@pytest.mark.parametrize(
    "call",
    [
        lambda c: c.abort(),
        lambda c: c.takeoff(1.0),
        lambda c: c.land(),
        lambda c: c.hold(),
        lambda c: c.set_position(1.0, 0.0, 0.0),
        lambda c: c.set_velocity(0.5, 0.0, 0.0),
        lambda c: c.set_yaw(1.0),
    ],
)
def test_setpoint_methods_raise_not_implemented_even_when_disarmed(call):
    # These stubs must raise NotImplementedError unconditionally -- not
    # as a side effect of an armed-state check that happens to run first.
    # If a future edit added a `self.armed` check ahead of the
    # `raise NotImplementedError`, calling one of these while disarmed
    # would start raising NotArmedError instead and this real "not built
    # yet" gap would go quiet.
    controller = MAVROSFlightController(FakeFlightCommandClient(is_armed=False))
    with pytest.raises(NotImplementedError):
        call(controller)
