"""Tests for geometry.py -- migrated from gps_denied/raj-dev's
test/test_geometry.py (see CHECKPOINT/CURRENT_STATE.md, NIDAR Autonomy
Migration). Pure logic, no ROS required."""
import math

from pytest import approx as pytest_approx

from nidar_autonomy.geometry import (
    RigidTransform2D,
    quaternion_from_yaw,
    wrap,
    yaw_from_quaternion,
)


class TestWrap:
    def test_identity_in_range(self):
        assert wrap(0.5) == pytest_approx(0.5)

    def test_wraps_positive_overflow(self):
        assert wrap(math.pi + 0.1) == pytest_approx(-math.pi + 0.1)

    def test_wraps_negative_overflow(self):
        assert wrap(-math.pi - 0.1) == pytest_approx(math.pi - 0.1)

    def test_large_multiple_of_tau(self):
        assert wrap(0.3 + 10 * math.tau) == pytest_approx(0.3)


class TestQuaternionYaw:
    def test_zero_yaw_roundtrip(self):
        q = quaternion_from_yaw(0.0)
        assert yaw_from_quaternion(*q) == pytest_approx(0.0, abs=1e-9)

    def test_roundtrip_various_angles(self):
        for deg in (-179, -90, -1, 0, 1, 45, 90, 135, 179):
            yaw = math.radians(deg)
            q = quaternion_from_yaw(yaw)
            assert yaw_from_quaternion(*q) == pytest_approx(yaw, abs=1e-9), f"failed at {deg} deg"

    def test_identity_quaternion_is_zero_yaw(self):
        assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest_approx(0.0)

    def test_90_degree_quaternion(self):
        # (0,0,sin(45deg),cos(45deg)) is a +90deg yaw quaternion.
        q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        assert yaw_from_quaternion(*q) == pytest_approx(math.pi / 2)


class TestRigidTransform2D:
    """Covers the frame-mixing bug class: map-frame reasoning must be
    correctly rotated+translated into the flight controller's local frame
    before being used for anything downstream, or "go 3m up the corridor"
    becomes "go 3m in some other direction." This class is telemetry/
    planning-support math only -- it never itself issues a flight command."""

    def test_identity_transform_when_frames_agree(self):
        t = RigidTransform2D(smoothing=1.0)  # snap fully each update for the test
        t.update(1.0, 2.0, 0.3, 1.0, 2.0, 0.3)
        t.update(1.0, 2.0, 0.3, 1.0, 2.0, 0.3)
        x, y, yaw = t.apply(5.0, 5.0, 0.0)
        assert x == pytest_approx(5.0)
        assert y == pytest_approx(5.0)
        assert yaw == pytest_approx(0.0)

    def test_pure_translation_offset(self):
        t = RigidTransform2D(smoothing=1.0)
        # Same physical point: frame A says (0,0), frame B says (10, -3).
        t.update(0.0, 0.0, 0.0, 10.0, -3.0, 0.0)
        t.update(0.0, 0.0, 0.0, 10.0, -3.0, 0.0)
        x, y, yaw = t.apply(2.0, 0.0, 0.0)
        # A frame-A point 2m ahead should land 2m ahead of the offset, not
        # be reinterpreted through some other rotation.
        assert x == pytest_approx(12.0)
        assert y == pytest_approx(-3.0)

    def test_pure_yaw_offset_rotates_direction_correctly(self):
        t = RigidTransform2D(smoothing=1.0)
        # The two frames are identical in origin but rotated 90 degrees from
        # each other -- this is the "frame mixing sends the drone sideways"
        # bug class this class exists to prevent.
        t.update(0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2)
        t.update(0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2)
        x, y, yaw = t.apply(1.0, 0.0, 0.0)  # "go 1m along frame-A +x"
        # In frame B that must become +y, not +x.
        assert x == pytest_approx(0.0)
        assert y == pytest_approx(1.0)

    def test_smoothing_converges_toward_true_offset(self):
        t = RigidTransform2D(smoothing=0.1)
        for _ in range(200):
            t.update(0.0, 0.0, 0.0, 5.0, 5.0, 0.0)
        assert t.x == pytest_approx(5.0, abs=1e-3)
        assert t.y == pytest_approx(5.0, abs=1e-3)

    def test_offset_magnitude(self):
        t = RigidTransform2D(smoothing=1.0)
        t.update(0.0, 0.0, 0.0, 3.0, 4.0, 0.0)
        assert t.offset_magnitude() == pytest_approx(5.0)

    def test_not_initialized_before_first_update(self):
        t = RigidTransform2D()
        assert not t.initialized
        t.update(0.0, 0.0, 0.0, 1.0, 1.0, 0.0)
        assert t.initialized
