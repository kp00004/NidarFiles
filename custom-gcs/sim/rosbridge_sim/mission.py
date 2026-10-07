"""Deterministic mission simulation core.

Pure function of (config, elapsed seconds since mission start) -> simulated
telemetry/map/survivor state. No clock, no networking, no knowledge of
"idle" or "aborted" — those are properties of *whether/when* a mission is
running, which is the server layer's job (see server.py's MissionControl).
This class only answers "what does a running mission look like at time t",
which keeps it trivially unit-testable.

Field shapes match docs/DATA_MODELS.md; grid/reveal logic matches
docs/DECISIONS.md D-7/D-7a (1 m x 1 m grid, origin at the entry point).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from . import grid as gridmod

# Fraction of mission duration spent in each progress phase, in order.
_ENTERING_FRACTION = 0.05
_EXITING_FRACTION = 0.90  # exiting starts here; "complete" at 1.0

# Battery drains from 100% toward this floor over the mission duration.
_BATTERY_END_FRACTION = 0.40
_VOLTAGE_FULL = 16.8  # 4S LiPo, fully charged
_VOLTAGE_AT_FLOOR = 14.8  # 4S LiPo, ~40% remaining


@dataclass(frozen=True)
class MissionConfig:
    width: int = 15
    height: int = 15
    resolution: float = 1.0
    duration_s: float = 300.0
    num_survivors: int = 6
    seed: int = 42

    def __post_init__(self) -> None:
        if not (0 < self.num_survivors <= 6):
            raise ValueError("num_survivors must be between 1 and 6 (Rulebook cap)")
        if self.duration_s <= 0:
            raise ValueError("duration_s must be positive")


@dataclass(frozen=True)
class SurvivorSpec:
    survivor_id: int
    row: int
    col: int
    detect_at_s: float
    confidence: float


def _cell_center_xy(row: int, col: int, resolution: float) -> tuple[float, float]:
    return (col + 0.5) * resolution, (row + 0.5) * resolution


class MissionSimulator:
    def __init__(self, config: MissionConfig) -> None:
        self.config = config
        self._ground_truth = gridmod.generate_ground_truth(config.width, config.height, config.seed)
        self._reveal_order = gridmod.compute_reveal_order(config.width, config.height, config.seed)
        self._survivors = self._place_survivors()

    # -- setup -------------------------------------------------------

    def _place_survivors(self) -> list[SurvivorSpec]:
        rng = random.Random(self.config.seed ^ 0x5AFE)
        free_cells = [
            (r, c)
            for r in range(self.config.height)
            for c in range(self.config.width)
            if self._ground_truth[r][c] == gridmod.FREE and (r, c) != self._reveal_order[0]
        ]
        if len(free_cells) < self.config.num_survivors:
            raise ValueError("arena too small/dense for the configured number of survivors")

        chosen = rng.sample(free_cells, k=self.config.num_survivors)
        # Spread detections across the "searching" portion of the mission.
        search_start = self.config.duration_s * _ENTERING_FRACTION
        search_end = self.config.duration_s * _EXITING_FRACTION
        span = max(search_end - search_start, 1e-6)

        specs = []
        for i, (r, c) in enumerate(chosen):
            fraction = (i + 1) / (self.config.num_survivors + 1)
            detect_at = search_start + fraction * span
            confidence = round(rng.uniform(0.75, 0.98), 2)
            specs.append(SurvivorSpec(survivor_id=i + 1, row=r, col=c, detect_at_s=detect_at, confidence=confidence))
        return specs

    # -- mission progress phase --------------------------------------

    def progress_phase_at(self, elapsed_s: float) -> str:
        """Phase of a *running* mission at `elapsed_s`. Does not know
        about "idle" or "aborted" — see MissionControl in server.py."""
        d = self.config.duration_s
        clamped = max(0.0, elapsed_s)
        if clamped >= d:
            return "complete"
        if clamped >= d * _EXITING_FRACTION:
            return "exiting"
        if clamped >= d * _ENTERING_FRACTION:
            return "searching"
        return "entering"

    def mission_state_msg_at(self, elapsed_s: float) -> dict:
        return {"data": self.progress_phase_at(elapsed_s)}

    # -- map -----------------------------------------------------------

    def revealed_cells_at(self, elapsed_s: float) -> set[gridmod.Cell]:
        total = len(self._reveal_order)
        fraction = min(max(elapsed_s, 0.0), self.config.duration_s) / self.config.duration_s
        count = max(1, round(total * fraction))
        return set(self._reveal_order[:count])

    def occupancy_grid_at(self, elapsed_s: float, *, stamp_sec: int = 0) -> dict:
        return gridmod.build_occupancy_grid_msg(
            self._ground_truth,
            self.revealed_cells_at(elapsed_s),
            resolution=self.config.resolution,
            width=self.config.width,
            height=self.config.height,
            stamp_sec=stamp_sec,
        )

    # -- pose ------------------------------------------------------------

    def pose_at(self, elapsed_s: float) -> dict:
        """Drone position/orientation: follows the reveal frontier, i.e.
        the drone is always at (or just behind) the most recently
        explored cell — deterministic, always consistent with the map."""
        revealed = self.revealed_cells_at(elapsed_s)
        current_index = len(revealed) - 1
        row, col = self._reveal_order[current_index]
        x, y = _cell_center_xy(row, col, self.config.resolution)

        if current_index > 0:
            prow, pcol = self._reveal_order[current_index - 1]
            heading_rad = _heading_between(prow, pcol, row, col)
        else:
            heading_rad = 0.0

        return {
            "header": {"stamp": {"sec": int(elapsed_s), "nanosec": 0}, "frame_id": "map"},
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": _yaw_to_quaternion(heading_rad),
            },
        }

    # -- battery -----------------------------------------------------

    def battery_at(self, elapsed_s: float) -> dict:
        fraction_elapsed = min(max(elapsed_s, 0.0), self.config.duration_s) / self.config.duration_s
        percentage = 1.0 - (1.0 - _BATTERY_END_FRACTION) * fraction_elapsed
        voltage = _VOLTAGE_FULL - (_VOLTAGE_FULL - _VOLTAGE_AT_FLOOR) * fraction_elapsed
        return {
            "header": {"stamp": {"sec": int(elapsed_s), "nanosec": 0}, "frame_id": "battery"},
            "voltage": round(voltage, 2),
            "percentage": round(percentage, 4),
        }

    # -- survivors -----------------------------------------------------

    # -- coverage / planned path / normalized telemetry -----------------
    #
    # Added alongside the NIDAR Autonomy Migration (see
    # CHECKPOINT/CURRENT_STATE.md) so custom-gcs's map/coverage/path/
    # autonomy-state rendering can be exercised end-to-end against this
    # simulator, the same way pose/battery/map already are. Deliberately
    # simple: coverage here just mirrors "revealed and free" (a real
    # coverage_tracker.py lags behind mapping and needs camera FOV/LOS --
    # see onboard-autonomy/nidar_autonomy/coverage_grid.py -- this
    # simulator does not attempt to reproduce that fidelity, only the
    # wire shape and basic reveal-over-time behavior a GCS panel needs to
    # be tested against).

    def coverage_grid_at(self, elapsed_s: float, *, stamp_sec: int = 0) -> dict:
        revealed = self.revealed_cells_at(elapsed_s)
        data = [gridmod.UNKNOWN] * (self.config.width * self.config.height)
        for r, c in revealed:
            if self._ground_truth[r][c] == gridmod.FREE:
                data[r * self.config.width + c] = 100  # SEARCHED
        return {
            "header": {"stamp": {"sec": stamp_sec, "nanosec": 0}, "frame_id": "map"},
            "info": {
                "resolution": self.config.resolution,
                "width": self.config.width,
                "height": self.config.height,
                "origin": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "data": data,
        }

    def planned_path_at(self, elapsed_s: float, *, stamp_sec: int = 0) -> dict:
        """A short nav_msgs/Path from the current simulated position toward
        the next few not-yet-revealed frontier cells -- just enough for a
        GCS panel to have something real to render, not a real planner
        output."""
        revealed = self.revealed_cells_at(elapsed_s)
        current_index = len(revealed) - 1
        lookahead = self._reveal_order[current_index : current_index + 6]
        poses = []
        for row, col in lookahead:
            x, y = _cell_center_xy(row, col, self.config.resolution)
            poses.append(
                {
                    "header": {"stamp": {"sec": stamp_sec, "nanosec": 0}, "frame_id": "map"},
                    "pose": {"position": {"x": x, "y": y, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
                }
            )
        return {
            "header": {"stamp": {"sec": stamp_sec, "nanosec": 0}, "frame_id": "map"},
            "poses": poses,
        }

    def telemetry_state_at(self, elapsed_s: float, phase: str) -> dict:
        """The normalized `/telemetry/state` contract (see
        CHECKPOINT/docs/gcs_telemetry_contract.md), built from this same
        simulator's already-deterministic state -- not a second source of
        truth, just a different shape over the same numbers."""
        revealed = self.revealed_cells_at(elapsed_s)
        total_cells = self.config.width * self.config.height
        explored_pct = round(100.0 * len(revealed) / total_cells, 1)
        pose = self.pose_at(elapsed_s)
        position = pose["pose"]["position"]

        current_index = len(revealed) - 1
        target = None
        if current_index + 1 < len(self._reveal_order):
            trow, tcol = self._reveal_order[current_index + 1]
            target = list(_cell_center_xy(trow, tcol, self.config.resolution))

        autonomy_state = {
            "idle": ("IDLE", "Awaiting mission start", "Wait for GCS start"),
            "entering": ("ENTERING", "Arm and enter the arena", "Hold for arming confirmation"),
            "searching": ("SEARCHING_FRONTIER", "Explore unexplored region", "Navigate to frontier"),
            "exiting": ("RETURNING_TO_ENTRY", "Return to entry/exit point", "Navigate to entry"),
            "complete": ("MISSION_COMPLETE", "Mission complete", "Land and disarm"),
            "aborted": ("ABORTED", "Mission aborted", "Hold -- awaiting ground reset"),
        }.get(phase, ("UNKNOWN", "Unknown", "Unknown"))

        return {
            "schema_version": 1,
            "stamp": elapsed_s,
            "connection": {"connected": True, "heartbeat_age_sec": 0.1},
            "flight": {"armed": phase not in ("idle", "aborted"), "mode": "GUIDED", "system_status": 4,
                       "battery_pct": self.battery_at(elapsed_s)["percentage"] * 100.0},
            "position": {"x": position["x"], "y": position["y"], "z": position["z"],
                        "yaw_deg": 0.0, "velocity_mps": 0.3},
            "sensors": {"slam": "ok", "lidar": "ok", "rangefinder": "not_integrated", "camera": "not_integrated"},
            "mapping": {
                "available": True,
                "resolution_m": self.config.resolution,
                "width_cells": self.config.width,
                "height_cells": self.config.height,
                "origin_x": 0.0,
                "origin_y": 0.0,
                "coverage_cell_size_m": self.config.resolution,
                "explored_pct": explored_pct,
            },
            "navigation": {
                "target": target,
                "frontier_count": 1 if target is not None else 0,
                "candidate_count": 1 if target is not None else 0,
                "blacklisted_count": 0,
                "path_progress": None,
                "brake_active": False,
                "escaping_local_minimum": False,
                "map_local_offset_m": 0.0,
                "geofence_breached": False,
            },
            "autonomy": {
                "state": autonomy_state[0],
                "objective": autonomy_state[1],
                "target": target,
                "next_action": autonomy_state[2],
            },
            "mission": {"state": phase, "elapsed_sec": elapsed_s, "complete": phase == "complete"},
            "survivors": [],
        }

    def survivors_detected_at(self, elapsed_s: float) -> list[dict]:
        detected = [s for s in self._survivors if s.detect_at_s <= elapsed_s]
        out = []
        for s in detected:
            x, y = _cell_center_xy(s.row, s.col, self.config.resolution)
            out.append({"survivor_id": s.survivor_id, "x": x, "y": y, "confidence": s.confidence})
        return out


def _heading_between(r0: int, c0: int, r1: int, c1: int) -> float:
    dx = c1 - c0
    dy = r1 - r0
    return math.atan2(dy, dx)


def _yaw_to_quaternion(yaw_rad: float) -> dict:
    half = yaw_rad / 2.0
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}
