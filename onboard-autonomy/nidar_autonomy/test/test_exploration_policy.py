"""Tests for exploration_policy.py -- the target-selection policy migrated/
adapted from gps_denied/raj-dev's frontier_explorer.py (NIDAR Autonomy
Migration). Pure logic, no ROS required."""
from nidar_autonomy.exploration_policy import (
    apply_sweep_radius,
    blacklist_key,
    clear_failure,
    is_blacklisted,
    record_failure,
    record_visit,
    score_candidates,
    select_target,
    visit_key,
    visit_penalty,
)
from nidar_autonomy.frontier_detector import FrontierCandidate


def _candidate(x, y, score=10.0, cell_count=5):
    return FrontierCandidate(x=x, y=y, score=score, cell_count=cell_count)


class TestVisitedMemory:
    def test_record_and_penalty_grows_with_visits(self):
        visited = {}
        record_visit(visited, 1.0, 1.0, cell_size=0.75)
        p1 = visit_penalty(visited, 1.0, 1.0, cell_size=0.75, radius=1.5, weight=0.6)
        record_visit(visited, 1.0, 1.0, cell_size=0.75)
        p2 = visit_penalty(visited, 1.0, 1.0, cell_size=0.75, radius=1.5, weight=0.6)
        assert p2 > p1 > 0.0

    def test_unvisited_area_has_zero_penalty(self):
        visited = {}
        assert visit_penalty(visited, 50.0, 50.0, cell_size=0.75, radius=1.5, weight=0.6) == 0.0

    def test_penalty_is_capped(self):
        visited = {}
        for _ in range(1000):
            record_visit(visited, 0.0, 0.0, cell_size=0.75)
        penalty = visit_penalty(visited, 0.0, 0.0, cell_size=0.75, radius=1.5, weight=1.0, cap=20.0)
        assert penalty == 20.0

    def test_visit_key_quantizes(self):
        assert visit_key(1.0, 1.0, 0.75) == visit_key(1.1, 1.1, 0.75)


class TestBlacklist:
    def test_not_blacklisted_below_threshold(self):
        bl = {}
        record_failure(bl, 3.0, 3.0)
        record_failure(bl, 3.0, 3.0)
        assert not is_blacklisted(bl, 3.0, 3.0, max_failures=3)

    def test_blacklisted_at_threshold(self):
        bl = {}
        for _ in range(3):
            record_failure(bl, 3.0, 3.0)
        assert is_blacklisted(bl, 3.0, 3.0, max_failures=3)

    def test_clear_failure_resets(self):
        bl = {}
        record_failure(bl, 3.0, 3.0, weight=5)
        clear_failure(bl, 3.0, 3.0)
        assert not is_blacklisted(bl, 3.0, 3.0, max_failures=1)

    def test_blacklist_key_rounds_to_one_decimal(self):
        assert blacklist_key(3.04, 3.04) == blacklist_key(3.02, 3.049)


class TestScoreCandidates:
    def test_closer_and_larger_scores_lower_cost(self):
        far_small = _candidate(10.0, 10.0, score=2.0, cell_count=1)
        near_large = _candidate(2.0, 0.0, score=20.0, cell_count=10)
        scored = score_candidates(
            [far_small, near_large], pose=(0.0, 0.0), current_goal=None,
            visited={}, blacklist={},
        )
        assert scored[0].x == near_large.x and scored[0].y == near_large.y

    def test_min_goal_distance_excludes_too_close_candidates(self):
        too_close = _candidate(0.1, 0.0)
        scored = score_candidates(
            [too_close], pose=(0.0, 0.0), current_goal=None,
            visited={}, blacklist={}, min_goal_distance=0.7,
        )
        assert scored == []

    def test_blacklisted_candidate_excluded(self):
        c = _candidate(5.0, 0.0)
        bl = {}
        for _ in range(3):
            record_failure(bl, c.x, c.y)
        scored = score_candidates(
            [c], pose=(0.0, 0.0), current_goal=None, visited={}, blacklist=bl, max_failures=3,
        )
        assert scored == []

    def test_hysteresis_keeps_current_goal_preferred(self):
        """A genuinely different, slightly-cheaper alternative should NOT
        beat the goal already being pursued once the hysteresis discount is
        applied -- this is the anti-flip-flop behavior gps_denied's
        frontier_explorer.py added after observing real thrashing between
        two candidates. `current` sits far enough from `alternative` that
        only `current` is within the "same goal" tolerance of
        `current_goal` (which coincides with `current`)."""
        current = _candidate(5.0, 0.0, score=10.0, cell_count=5)  # cost 5 - 0.4*10 = 1.0
        alternative = _candidate(6.0, 0.0, score=15.0, cell_count=7)  # cost 6 - 0.4*15 = 0.0

        # Without hysteresis, alternative wins (strictly cheaper).
        scored_no_hysteresis = score_candidates(
            [current, alternative], pose=(0.0, 0.0), current_goal=None,
            visited={}, blacklist={}, hysteresis_bonus=2.0,
        )
        assert scored_no_hysteresis[0].x == alternative.x

        # With current_goal coinciding with `current`, only `current` gets
        # the discount (alternative is 1.0m away -- not "the same goal") and
        # the discount (2.0) exceeds alternative's cost advantage (1.0), so
        # the already-committed goal wins.
        scored_with_hysteresis = score_candidates(
            [current, alternative], pose=(0.0, 0.0), current_goal=(current.x, current.y),
            visited={}, blacklist={}, hysteresis_bonus=2.0,
        )
        assert scored_with_hysteresis[0].x == current.x

    def test_visited_area_scores_worse(self):
        c = _candidate(5.0, 0.0, score=10.0, cell_count=5)
        visited = {}
        for _ in range(20):
            record_visit(visited, c.x, c.y, cell_size=0.75)
        scored_visited = score_candidates(
            [c], pose=(0.0, 0.0), current_goal=None, visited=visited, blacklist={},
        )
        scored_fresh = score_candidates(
            [c], pose=(0.0, 0.0), current_goal=None, visited={}, blacklist={},
        )
        assert scored_visited[0].cost > scored_fresh[0].cost

    def test_empty_candidates_returns_empty(self):
        assert score_candidates([], pose=(0.0, 0.0), current_goal=None, visited={}, blacklist={}) == []


class TestSweepRadius:
    def test_prefers_near_candidates_when_available(self):
        from nidar_autonomy.exploration_policy import ScoredTarget

        near = ScoredTarget(x=1.0, y=0.0, cost=5.0, cell_count=3, distance=1.0)
        far = ScoredTarget(x=20.0, y=0.0, cost=1.0, cell_count=3, distance=20.0)  # cheaper but far
        result = apply_sweep_radius([far, near], sweep_radius=5.0)
        assert result == [near]

    def test_falls_back_to_all_when_nothing_near(self):
        from nidar_autonomy.exploration_policy import ScoredTarget

        far = ScoredTarget(x=20.0, y=0.0, cost=1.0, cell_count=3, distance=20.0)
        result = apply_sweep_radius([far], sweep_radius=5.0)
        assert result == [far]


class TestSelectTarget:
    def test_returns_none_when_no_candidates(self):
        assert select_target([], pose=(0.0, 0.0), current_goal=None, visited={}, blacklist={}) is None

    def test_returns_none_when_all_blacklisted(self):
        c = _candidate(5.0, 0.0)
        bl = {}
        for _ in range(3):
            record_failure(bl, c.x, c.y)
        assert select_target(
            [c], pose=(0.0, 0.0), current_goal=None, visited={}, blacklist=bl, max_failures=3
        ) is None

    def test_selects_best_reachable_candidate(self):
        c1 = _candidate(5.0, 0.0, score=5.0, cell_count=2)
        c2 = _candidate(2.0, 0.0, score=20.0, cell_count=10)
        target = select_target([c1, c2], pose=(0.0, 0.0), current_goal=None, visited={}, blacklist={})
        assert target is not None
        assert (target.x, target.y) == (c2.x, c2.y)
