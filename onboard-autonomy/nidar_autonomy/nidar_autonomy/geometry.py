"""2D pose/frame math shared across the mapping/exploration stack.

Migrated from gps_denied/raj-dev's scripts/lib/geometry.py (NIDAR Autonomy
Migration, see CHECKPOINT/CURRENT_STATE.md). Ported unchanged -- this module
had no rclpy dependency there and needs none here either.

All angles are radians. Quaternions are passed as plain (x, y, z, w) floats,
never as geometry_msgs objects, so this module stays importable/testable
without a ROS environment, same posture as flight_command_interface.py and
indoor_environment.py.
"""
from __future__ import annotations

import math


def wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about Z) from a quaternion, ignoring roll/pitch --
    correct for a planar 2D pose where only heading matters."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Build a (x, y, z, w) quaternion representing a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RigidTransform2D:
    """A smoothed 2D rigid transform (yaw, tx, ty) taking frame A -> frame B.

    Used to relate two independent pose estimates of the SAME physical body
    that live in different frames -- e.g. a SLAM map frame and the flight
    controller's local EKF frame. Comparing the two live poses gives the
    offset directly; it is low-pass filtered because the true offset is
    near-constant while both pose inputs are individually noisy, and a
    jittery transform would shake any setpoint built from it.

    This is intentionally generic: it doesn't know which frame is "map" and
    which is "local," or which flight controller is on the other end. Any
    two co-located pose streams can be related this way.

    Note: this class only ever *estimates and applies* a frame transform for
    telemetry/planning purposes -- it never itself issues a flight command.

    NOT YET WIRED TO A CALLER (2026-09-09, NIDAR Autonomy Migration): this
    class is migrated and tested, but its intended consumer --
    `vision_pose_bridge_node.py` (SLAM pose -> FCU EKF input) -- was
    deliberately deferred this session (see
    CHECKPOINT/CURRENT_STATE.md §24's migration matrix: it feeds the FCU's
    EKF, a step closer to flight-relevant than pure telemetry, and was
    scoped out rather than rushed). Wire this class in when that node is
    built, or when `frontier_explorer_node.py`/`coverage_tracker_node.py`
    need to reconcile a second pose source -- do not read its absence of a
    current caller as dead code.
    """

    def __init__(self, smoothing: float = 0.05) -> None:
        self.smoothing = smoothing
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.initialized = False

    def update(self, ax: float, ay: float, ayaw: float, bx: float, by: float, byaw: float) -> None:
        """Feed one simultaneous sample of (pose in A, pose in B)."""
        dyaw = wrap(byaw - ayaw)
        c, s = math.cos(dyaw), math.sin(dyaw)
        tx = bx - (c * ax - s * ay)
        ty = by - (s * ax + c * ay)

        if not self.initialized:
            self.yaw, self.x, self.y = dyaw, tx, ty
            self.initialized = True
            return

        a = self.smoothing
        self.yaw = wrap(self.yaw + a * wrap(dyaw - self.yaw))
        self.x += a * (tx - self.x)
        self.y += a * (ty - self.y)

    def apply(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        """Map a pose from frame A into frame B using the current estimate."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * x - s * y + self.x,
                s * x + c * y + self.y,
                wrap(yaw + self.yaw))

    def offset_magnitude(self) -> float:
        """Translational offset magnitude -- large + settled means the two
        pose sources have diverged (e.g. SLAM/EKF divergence)."""
        return math.hypot(self.x, self.y)
