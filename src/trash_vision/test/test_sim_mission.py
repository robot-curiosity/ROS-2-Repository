import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from trash_vision import sim_mission as sm  # noqa: E402

# Software target toward the 80% field goal (sim only, not a robot result).
MIN_FRACTION = 0.8


def test_open_floor_pickup_rate():
    r = sm.run_mission(sm.OPEN_LITTER)
    print("open-floor sim pickup rate: %.0f%% (%d/%d) in %.0f s"
          % (100 * r["fraction"], r["collected"], r["total"], r["duration"]))
    assert r["fraction"] >= MIN_FRACTION


def test_near_wall_pickup_rate():
    r = sm.run_mission(sm.WALL_LITTER)
    print("near-wall sim pickup rate: %.0f%% (%d/%d)"
          % (100 * r["fraction"], r["collected"], r["total"]))
    assert r["fraction"] >= MIN_FRACTION


def test_combined_scenario_every_state_visited():
    r = sm.run_mission(sm.OPEN_LITTER + sm.WALL_LITTER)
    print("combined sim pickup rate: %.0f%%" % (100 * r["fraction"]))
    assert r["fraction"] >= MIN_FRACTION
    for state in ("SEARCH", "APPROACH", "PICK", "DEPOSIT", "STOW"):
        assert r["states"].get(state, 0) > 0


def test_corner_litter_baseline():
    r = sm.run_mission(sm.CORNER_LITTER)
    print("corner sim pickup rate: %.0f%%" % (100 * r["fraction"]))
    assert r["fraction"] >= 0.5  # current baseline, must not regress


def test_side_sensors_matter():
    # Refusing every spin (huge side_clearance) must hurt near walls, so the
    # side-sensor logic is what the near-wall scenario exercises.
    base = sm.run_mission(sm.WALL_LITTER)["fraction"]
    blind = sm.run_mission(sm.WALL_LITTER,
                           params={"side_clearance": 5.0})["fraction"]
    assert blind < base


def test_no_litter_is_trivially_complete():
    r = sm.run_mission([], duration=5.0)
    assert r["total"] == 0 and r["fraction"] == 1.0


def test_unreachable_litter_times_out_without_crash():
    r = sm.run_mission([(9.0, 9.0)], duration=20.0)
    assert r["collected"] == 0 and r["duration"] >= 20.0
