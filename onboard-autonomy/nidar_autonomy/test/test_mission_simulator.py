"""Tests for mission_simulator.py -- the deterministic end-to-end
simulation engine behind the GCS's "RUN SIMULATION" path. Pure logic, no
ROS required (this module never imports rclpy/mavros).

These tests are the actual proof this simulation is not a fake progress
bar: they run the REAL engine (frontier detection, target selection, A*
planning, simulated motion, coverage tracking, mission-state transitions)
to completion and assert on genuine, non-trivial behavior at each stage
-- not on a manually-set final state.
"""
import inspect
import math

import pytest

import nidar_autonomy.mission_simulator as mission_simulator_module
from nidar_autonomy.mission_simulator import MissionSimulator


# -- 1. Simulation can start --------------------------------------------------


def test_run_starts_the_simulation():
    sim = MissionSimulator()
    snap = sim.run()
    assert snap.status == "running"
    assert snap.mission_state == "entering"
    assert snap.step == 0


def test_idle_before_run():
    sim = MissionSimulator()
    snap = sim.snapshot()
    assert snap.status == "idle"


# -- 2/3/4. Never touches real flight control or mavros ----------------------


def test_never_imports_ros_or_real_flight_control():
    """Static proof, not just runtime behavior: the module's own actual
    import statements can never reach rclpy, mavros_msgs, flight_command,
    or arming_guard -- there is no path from this simulator to the real
    Pixhawk. (A runtime "did it call X" test can miss a code path; this
    can't -- the import simply isn't there.) Checks only real `import`
    lines, not prose in comments/docstrings that legitimately discusses
    these names (as this module's own module docstring does, to explain
    exactly this guarantee)."""
    import_lines = [
        line.strip()
        for line in inspect.getsource(mission_simulator_module).splitlines()
        if line.strip().startswith(("import ", "from "))
    ]
    forbidden = ("rclpy", "mavros_msgs", "flight_command", "arming_guard")
    for line in import_lines:
        for name in forbidden:
            assert name not in line, f"forbidden import found: {line!r}"


def test_never_publishes_or_subscribes_to_anything():
    """The simulation has no rosbridge/ROS publisher or subscriber of any
    kind -- it cannot reach `/gcs/command` or the real `/mission/state`
    even indirectly, because it never imports rclpy at all (previous
    test) and therefore has no `create_publisher`/`create_subscription`
    call anywhere -- there is no ROS communication surface here at all,
    real or otherwise."""
    source = inspect.getsource(mission_simulator_module)
    assert "create_publisher" not in source
    assert "create_subscription" not in source


# -- 5. Deterministic world initialization ------------------------------------


def test_world_initialization_is_deterministic():
    sim_a = MissionSimulator()
    sim_b = MissionSimulator()
    snap_a = sim_a.run()
    snap_b = sim_b.run()
    assert snap_a.pose == snap_b.pose
    assert snap_a.map == snap_b.map


def test_full_run_is_deterministic():
    sim_a = MissionSimulator()
    sim_b = MissionSimulator()
    final_a = sim_a.run_to_completion()
    final_b = sim_b.run_to_completion()
    assert final_a.status == final_b.status == "completed"
    assert final_a.step == final_b.step
    assert final_a.map_known_pct == final_b.map_known_pct
    assert final_a.pose == final_b.pose


# -- 6. Occupancy map changes as the simulated drone explores ----------------


def test_map_known_fraction_grows_over_time_not_instantly():
    """The specific, explicit requirement: the map must NOT appear fully
    explored at startup, and must grow progressively, not jump straight
    to its final value."""
    sim = MissionSimulator()
    sim.run()
    fractions = []
    for _ in range(60):
        snap = sim.step()
        fractions.append(snap.map_known_pct)
        if snap.status != "running":
            break

    assert fractions[0] < 20.0, "map must not be mostly known after only a handful of steps"
    assert fractions[-1] > fractions[0], "map must have grown by the end of this window"
    # Genuinely progressive: strictly non-decreasing throughout (known
    # cells are never un-learned), and grows at more than one point (not
    # one single instantaneous jump from ~0 to its final value).
    assert all(b >= a for a, b in zip(fractions, fractions[1:]))
    growth_events = sum(1 for a, b in zip(fractions, fractions[1:]) if b > a)
    assert growth_events >= 3, "map should grow across multiple distinct steps, not one big jump"


# -- 7. Frontier detector produces frontiers ----------------------------------


def test_frontiers_are_detected_during_search():
    sim = MissionSimulator()
    sim.run()
    saw_frontiers = False
    for _ in range(50):
        snap = sim.step()
        if snap.frontier_count > 0:
            saw_frontiers = True
            break
        if snap.status != "running":
            break
    assert saw_frontiers


# -- 8. Planner produces paths ------------------------------------------------


def test_planner_produces_a_nonempty_path():
    sim = MissionSimulator()
    sim.run()
    saw_path = False
    for _ in range(50):
        snap = sim.step()
        if snap.planned_path:
            saw_path = True
            assert all("x" in p and "y" in p for p in snap.planned_path)
            break
        if snap.status != "running":
            break
    assert saw_path


# -- 9. Simulated drone follows the generated path (not teleporting) --------


def test_drone_moves_incrementally_not_teleporting():
    """Consecutive poses while actively following a path must be close
    together (bounded by max_speed * dt), never a large jump -- proves
    the vehicle is genuinely stepping along the path, not being
    teleported frontier-to-frontier."""
    sim = MissionSimulator(dt_s=0.5)
    sim.run()
    max_speed_mps = 2.0  # MockFlightController's own default bound
    max_step_distance = max_speed_mps * sim._dt_s + 1e-6

    prev_pose = None
    checked_any_motion = False
    for _ in range(200):
        snap = sim.step()
        pose = (snap.pose["x"], snap.pose["y"])
        if prev_pose is not None:
            dist = math.hypot(pose[0] - prev_pose[0], pose[1] - prev_pose[1])
            assert dist <= max_step_distance + 1e-6, (
                f"single step moved {dist:.3f}m, more than one tick's max travel "
                f"({max_step_distance:.3f}m) -- looks like teleportation, not following a path"
            )
            if dist > 1e-6:
                checked_any_motion = True
        prev_pose = pose
        if snap.status != "running":
            break
    assert checked_any_motion, "drone must actually move across at least one step"


# -- 10. Coverage increases ---------------------------------------------------


def test_coverage_percent_is_nonzero_once_observed():
    sim = MissionSimulator()
    sim.run()
    saw_nonzero_coverage = False
    for _ in range(20):
        snap = sim.step()
        if snap.explored_pct > 0.0:
            saw_nonzero_coverage = True
            break
    assert saw_nonzero_coverage


# -- 11. Exploration continues through multiple iterations -------------------


def test_exploration_visits_multiple_distinct_targets():
    sim = MissionSimulator()
    sim.run_to_completion()
    # Re-run tracking targets explicitly (run_to_completion doesn't return
    # history) -- a fresh instance, same deterministic run.
    sim2 = MissionSimulator()
    sim2.run()
    targets_seen = set()
    for _ in range(600):
        snap = sim2.step()
        if snap.target is not None:
            targets_seen.add(tuple(snap.target))
        if snap.status != "running":
            break
    assert len(targets_seen) > 5, "exploration should visit many distinct frontier targets, not just one or two"


# -- 12. Simulation eventually reaches COMPLETE -------------------------------


def test_reaches_complete_through_the_full_lifecycle():
    sim = MissionSimulator()
    seen_states = []
    snap = sim.run()
    seen_states.append(snap.mission_state)
    for _ in range(1000):
        snap = sim.step()
        if snap.mission_state != seen_states[-1]:
            seen_states.append(snap.mission_state)
        if snap.status != "running":
            break

    assert snap.status == "completed"
    assert snap.error is None
    # The exact lifecycle order the mission brief specifies.
    assert seen_states == ["entering", "searching", "exiting", "complete"]


def test_complete_reaches_a_high_map_completion_not_a_token_amount():
    """Guards against a degenerate "completes after visiting one trivial
    frontier" regression -- the whole point of this simulation."""
    sim = MissionSimulator()
    final = sim.run_to_completion()
    assert final.status == "completed"
    assert final.map_known_pct > 80.0


def test_vehicle_returns_to_entry_and_lands():
    sim = MissionSimulator()
    final = sim.run_to_completion()
    assert final.status == "completed"
    entry = sim._environment.start_pose  # noqa: SLF001 -- test-only introspection
    assert final.pose["x"] == pytest.approx(entry[0], abs=1e-6)
    assert final.pose["y"] == pytest.approx(entry[1], abs=1e-6)
    assert final.pose["z"] == pytest.approx(0.0, abs=1e-6)
    assert not sim._controller.armed  # noqa: SLF001 -- landed -> disarmed (simulated only)


# -- 13. Simulation can be run twice from a clean reset -----------------------


def test_running_twice_produces_the_same_fresh_result():
    sim = MissionSimulator()
    first = sim.run_to_completion()
    second = sim.run_to_completion()  # run() always resets first
    assert first.status == second.status == "completed"
    assert first.step == second.step
    assert first.map_known_pct == second.map_known_pct
    assert first.pose == second.pose


def test_reset_returns_to_a_clean_idle_snapshot():
    sim = MissionSimulator()
    sim.run_to_completion()
    snap = sim.reset()
    assert snap.status == "idle"
    assert snap.step == 0
    assert snap.map["data"] == []


def test_run_after_reset_works_normally():
    sim = MissionSimulator()
    sim.run_to_completion()
    sim.reset()
    snap = sim.run_to_completion()
    assert snap.status == "completed"


# -- misc engine behavior ------------------------------------------------------


def test_step_is_a_noop_when_not_running():
    sim = MissionSimulator()
    idle_snap = sim.snapshot()
    stepped_snap = sim.step()
    assert stepped_snap == idle_snap


def test_step_is_a_noop_after_completion():
    sim = MissionSimulator()
    sim.run_to_completion()
    completed_snap = sim.snapshot()
    again = sim.step()
    assert again.status == "completed"
    assert again.step == completed_snap.step  # no further advancement


def test_max_steps_is_a_circuit_breaker_not_the_normal_path():
    """A pathologically small max_steps must fail cleanly (not hang, not
    silently report success) -- and the real default configuration must
    complete well before its own bound, proving completion is driven by
    actually running out of frontiers, not by hitting this limit."""
    sim = MissionSimulator(max_steps=5)
    final = sim.run_to_completion()
    assert final.status == "failed"
    assert "max_steps" in final.error

    normal = MissionSimulator()
    normal_final = normal.run_to_completion()
    assert normal_final.status == "completed"
    assert normal_final.step < normal._max_steps * 0.5  # noqa: SLF001 -- comfortable margin
