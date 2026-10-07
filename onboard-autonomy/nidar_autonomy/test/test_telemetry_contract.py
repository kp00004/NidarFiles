"""Tests for telemetry_contract.py -- migrated from gps_denied/raj-dev's
test/test_telemetry.py (NIDAR Autonomy Migration), adapted to
onboard-autonomy's own mission-state vocabulary
(topics.MISSION_STATES: idle/entering/searching/exiting/complete/aborted)
instead of gps_denied's mission_fsm.py states (not migrated -- see the
migration report). Pure logic, no ROS required."""
import json

from nidar_autonomy.telemetry_contract import (
    SCHEMA_VERSION,
    autonomy_state,
    build_contract,
    stale,
)
from nidar_autonomy.topics import MISSION_STATES


class TestAutonomyState:
    def test_exploring(self):
        s = autonomy_state("searching", "exploring", [1.0, 2.0])
        assert s["state"] == "SEARCHING_FRONTIER"
        assert s["target"] == [1.0, 2.0]
        assert s["objective"] == "Explore unexplored region"
        assert s["next_action"] == "Navigate to frontier"

    def test_returning_overrides_mission_state(self):
        # mission_state_node may still say "searching" while an exploration
        # node has already committed to returning -- the explorer's state is
        # more specific and must win.
        s = autonomy_state("searching", "returning", [0.0, 0.0])
        assert s["state"] == "RETURNING_TO_ENTRY"
        assert s["objective"] == "Return to entry/exit point"

    def test_done(self):
        s = autonomy_state("exiting", "done", None)
        assert s["state"] == "MISSION_COMPLETE"
        assert s["objective"] == "Mission complete"
        assert s["target"] is None

    def test_aborted_with_no_explorer_status_yet(self):
        s = autonomy_state("aborted", None, None)
        assert s["state"] == "ABORTED"
        assert s["objective"] == "Mission aborted"

    def test_unknown_mission_state_does_not_crash(self):
        s = autonomy_state(None, None, None)
        assert s["state"] == "UNKNOWN"
        assert s["objective"] == "Unknown"

    def test_every_mission_state_has_a_mapped_objective(self):
        for mission_state in MISSION_STATES:
            s = autonomy_state(mission_state, None, None)
            assert s["objective"] != "Unknown"
            assert s["next_action"] != "Unknown"

    def test_no_chain_of_thought_only_fixed_vocabulary(self):
        # The Active Thinking panel must never contain free-form text --
        # every state label must come from the fixed vocabulary below.
        allowed = {
            "WAITING_FOR_MAP", "SEARCHING_FRONTIER", "RETURNING_TO_ENTRY",
            "MISSION_COMPLETE", "IDLE", "ENTERING", "SEARCHING", "EXITING",
            "COMPLETE", "ABORTED", "UNKNOWN",
        }
        for mission_state in (None, *MISSION_STATES):
            for explorer_state in (None, "waiting", "exploring", "returning", "done"):
                s = autonomy_state(mission_state, explorer_state, None)
                assert s["state"] in allowed


class TestStale:
    def test_never_seen_is_stale(self):
        assert stale(None, 100.0, 3.0)

    def test_recent_is_not_stale(self):
        assert not stale(99.0, 100.0, 3.0)

    def test_old_is_stale(self):
        assert stale(90.0, 100.0, 3.0)

    def test_exactly_at_timeout_is_not_stale(self):
        assert not stale(97.0, 100.0, 3.0)


class TestBuildContract:
    def _minimal(self, **overrides):
        base = dict(
            connected=True, heartbeat_age_sec=0.5, armed=True, mode="GUIDED",
            system_status=4, battery_pct=87.0,
            position={"x": 1.0, "y": 2.0, "z": 1.2, "yaw_deg": 0.0, "velocity_mps": 0.3},
            sensors={"slam": "ok", "lidar": "ok", "rangefinder": "not_integrated",
                     "camera": "not_integrated"},
            mapping={"available": True, "resolution_m": 0.05, "width_cells": 100,
                     "height_cells": 100, "origin_x": 0.0, "origin_y": 0.0,
                     "coverage_cell_size_m": 1.0, "explored_pct": 42.0},
            navigation={"target": [1.0, 2.0]},
            autonomy=autonomy_state("searching", "exploring", [1.0, 2.0]),
            mission={"state": "searching", "elapsed_sec": 12.0, "complete": False},
            survivors=[],
        )
        base.update(overrides)
        return build_contract(**base)

    def test_has_schema_version_and_stamp(self):
        c = self._minimal()
        assert c["schema_version"] == SCHEMA_VERSION
        assert "stamp" in c

    def test_top_level_categories_present(self):
        c = self._minimal()
        for key in ("connection", "flight", "position", "sensors", "mapping",
                    "navigation", "autonomy", "mission", "survivors"):
            assert key in c

    def test_survivors_defaults_to_empty_list_not_fabricated(self):
        c = self._minimal()
        assert c["survivors"] == []

    def test_json_serializable(self):
        json.dumps(self._minimal())  # must not raise

    def test_flight_block_carries_battery(self):
        c = self._minimal(battery_pct=55.0)
        assert c["flight"]["battery_pct"] == 55.0
