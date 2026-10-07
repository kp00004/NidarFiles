import pytest

from nidar_autonomy.indoor_environment import empty_room, room_with_single_obstacle
from nidar_autonomy.path_planner_interface import (
    EndpointOccupiedError,
    EndpointOutOfBoundsError,
    NoPathFoundError,
    PathPlanner,
    PathPlanningError,
)


def test_interface_cannot_be_instantiated_directly():
    with pytest.raises(TypeError):
        PathPlanner()  # abstract -- must fail loud, not silently


def test_incomplete_implementation_cannot_be_instantiated():
    class Incomplete(PathPlanner):
        # deliberately missing plan()
        pass

    with pytest.raises(TypeError):
        Incomplete()


def test_exception_hierarchy():
    assert issubclass(EndpointOutOfBoundsError, PathPlanningError)
    assert issubclass(EndpointOccupiedError, PathPlanningError)
    assert issubclass(NoPathFoundError, PathPlanningError)
    # the three concrete cases must stay distinguishable from each other
    assert not issubclass(EndpointOutOfBoundsError, EndpointOccupiedError)
    assert not issubclass(EndpointOccupiedError, EndpointOutOfBoundsError)
    assert not issubclass(NoPathFoundError, EndpointOutOfBoundsError)
    assert not issubclass(NoPathFoundError, EndpointOccupiedError)


# -- _validate_endpoints, exercised via a minimal concrete planner ----------
#
# PathPlanner itself has no algorithm, only the shared validation helper
# every concrete planner is expected to call first -- exercised here
# directly (not via GridAStarPlanner) so this test only ever fails for a
# validation-logic regression, never a search-algorithm one.


class _ValidateOnlyPlanner(PathPlanner):
    """Minimal concrete planner: validates, then returns a trivial
    straight-line path without doing any real search -- exists only to
    exercise `PathPlanner._validate_endpoints` in isolation."""

    def plan(self, start, goal, environment):
        self._validate_endpoints(start, goal, environment)
        return [start, goal]


def test_validate_rejects_start_out_of_bounds():
    env = empty_room()
    planner = _ValidateOnlyPlanner()
    with pytest.raises(EndpointOutOfBoundsError):
        planner.plan((-1.0, -1.0), (5.0, 5.0), env)


def test_validate_rejects_goal_out_of_bounds():
    env = empty_room()
    planner = _ValidateOnlyPlanner()
    with pytest.raises(EndpointOutOfBoundsError):
        planner.plan((0.5, 0.5), (100.0, 100.0), env)


def test_validate_rejects_start_occupied():
    env = room_with_single_obstacle()
    planner = _ValidateOnlyPlanner()
    with pytest.raises(EndpointOccupiedError):
        planner.plan((7.5, 5.0), (1.5, 1.5), env)


def test_validate_rejects_goal_occupied():
    env = room_with_single_obstacle()
    planner = _ValidateOnlyPlanner()
    with pytest.raises(EndpointOccupiedError):
        planner.plan(env.start_pose, (7.5, 5.0), env)


def test_validate_out_of_bounds_takes_priority_over_occupied():
    # an out-of-bounds point is also reported occupied by
    # IndoorEnvironment.is_occupied -- validation must still distinguish
    # "not in the arena" from "in the arena but occupied" by checking
    # bounds first.
    env = empty_room()
    planner = _ValidateOnlyPlanner()
    with pytest.raises(EndpointOutOfBoundsError):
        planner.plan((-5.0, -5.0), (0.5, 0.5), env)


def test_validate_accepts_valid_distinct_endpoints():
    env = empty_room()
    planner = _ValidateOnlyPlanner()
    assert planner.plan(env.start_pose, (10.0, 10.0), env) == [env.start_pose, (10.0, 10.0)]
