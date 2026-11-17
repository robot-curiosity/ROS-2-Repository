import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from trash_vision import behavior_logic as bl  # noqa: E402
from trash_vision import search_pattern as sp  # noqa: E402
from trash_vision import sim_mission as sm  # noqa: E402

CLEAR = [math.inf, math.inf]


def test_row_spacing_from_fov_matches_geometry():
    s = sp.row_spacing_from_fov(66.0, 1.5, 0.2)
    assert abs(s - 2 * 1.5 * math.tan(math.radians(33)) * 0.8) < 1e-9
    assert sp.row_spacing_from_fov(66.0, 0.01, 0.0) >= 0.2


def test_waypoints_stay_inside_area_and_alternate():
    pts = sp.build_waypoints((0.0, 0.0), 4.0, 4.0, 1.0, 0.3)
    assert len(pts) >= 6
    for x, y in pts:
        assert abs(x) <= 1.7 + 1e-9 and abs(y) <= 1.7 + 1e-9
    # Row ends alternate: x flips sign every waypoint.
    for a, b in zip(pts, pts[1:]):
        if a[1] == b[1]:
            assert a[0] == -b[0]
    ys = sorted({y for _, y in pts})
    assert all(b - a <= 1.0 + 1e-9 for a, b in zip(ys, ys[1:]))


def test_waypoints_start_at_nearest_corner():
    pts = sp.build_waypoints((1.5, 1.5), 4.0, 4.0, 1.0, 0.3)
    assert pts[0][0] > 0 and pts[0][1] > 0
    pts = sp.build_waypoints((-1.5, -1.5), 4.0, 4.0, 1.0, 0.3)
    assert pts[0][0] < 0 and pts[0][1] < 0


def test_tiny_area_does_not_crash():
    pts = sp.build_waypoints((0.0, 0.0), 0.2, 0.2, 1.0, 0.3)
    assert pts and all(p == (0.0, 0.0) for p in pts)


def test_controller_turns_in_place_when_facing_away():
    s = sp.LawnmowerSearch({"search_area_length": 4.0,
                            "search_area_width": 4.0})
    lin, ang = s.step((0.0, 0.0, math.pi), 0.0)
    assert lin == 0.0 and ang != 0.0


def test_controller_drives_forward_when_aligned():
    s = sp.LawnmowerSearch()
    s.waypoints = [(2.0, 0.0), (2.0, 1.0)]
    lin, ang = s.step((0.0, 0.0, 0.0), 0.0)
    assert lin > 0 and abs(ang) < 1e-9


def test_waypoint_reached_advances_and_laps():
    s = sp.LawnmowerSearch()
    s.waypoints = [(1.0, 0.0), (0.0, 0.0)]
    s.step((1.0, 0.0, 0.0), 0.0)
    assert s.index == 1 and s.laps == 0
    s.step((0.0, 0.0, 0.0), 1.0)
    assert s.index == 0 and s.laps == 1


def test_stuck_waypoint_is_skipped():
    s = sp.LawnmowerSearch({"search_stuck_timeout": 5.0})
    s.waypoints = [(2.0, 0.0), (0.0, 2.0)]
    for t in range(0, 8):
        s.step((0.0, 0.0, 0.0), float(t))   # robot never moves
    assert s.skipped == 1 and s.index == 1


def test_blocked_front_skips_waypoint_after_timeout():
    s = sp.LawnmowerSearch({"search_stuck_timeout": 3.0})
    s.waypoints = [(2.0, 0.0), (0.0, 2.0)]
    s.step((0.0, 0.0, 0.0), 0.0)
    for t in range(1, 6):
        s.note_blocked(float(t))
    assert s.skipped == 1 and s.index == 1


def test_machine_spins_without_pose():
    m = bl.BehaviorMachine()
    cmd = m.step(0.0, None, CLEAR, CLEAR)
    assert cmd.linear == 0.0 and cmd.angular == m.p["search_turn_speed"]


def test_machine_follows_pattern_with_pose():
    m = bl.BehaviorMachine()
    cmds = [m.step(0.1 * i, None, CLEAR, CLEAR, pose=(0.0, 0.0, 0.0))
            for i in range(3)]
    assert m.state == bl.SEARCH
    assert any(c.linear > 0 or c.angular != 0 for c in cmds)
    assert m.search.target is not None


def test_machine_spin_mode_ignores_pose():
    m = bl.BehaviorMachine({"search_pattern": "spin"})
    cmd = m.step(0.0, None, CLEAR, CLEAR, pose=(0.0, 0.0, 0.0))
    assert cmd.linear == 0.0 and cmd.angular == m.p["search_turn_speed"]


def test_machine_avoids_obstacle_in_front_while_searching():
    m = bl.BehaviorMachine()
    cmd = m.step(0.0, None, [0.1, 0.5], CLEAR, pose=(0.0, 0.0, 0.0))
    assert cmd.linear == 0.0 and cmd.angular != 0.0


def test_sim_search_covers_most_of_area():
    r = sm.run_search_only(duration=300.0)
    print("sim search coverage: %.0f%%" % (100 * r["coverage"]))
    assert r["coverage"] >= 0.9


def test_sim_search_covers_area_with_wide_rows_less():
    wide = sm.run_search_only(duration=300.0,
                              params={"search_row_spacing": 3.0})
    full = sm.run_search_only(duration=300.0)
    assert wide["coverage"] < full["coverage"]


def test_sim_mission_reports_coverage():
    r = sm.run_mission(sm.OPEN_LITTER)
    print("open-floor coverage at finish: %.0f%%" % (100 * r["coverage"]))
    assert 0.0 < r["coverage"] <= 1.0


def test_coverage_grid_marks_view_wedge_only():
    g = sp.CoverageGrid(-2.0, 2.0, -2.0, 2.0)
    g.mark((0.0, 0.0, 0.0), 1.0, 66.0)
    f = g.fraction()
    assert 0.0 < f < 0.1
    g.mark((0.0, 0.0, math.pi), 1.0, 66.0)
    assert g.fraction() > f
