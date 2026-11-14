import pytest

from trash_vision import arm_logic as al


def test_clamp_pose():
    limits = [(-10, 10)] * 6
    assert al.clamp_pose([-20, 0, 20, 5, -5, 10], limits) == \
        [-10, 0, 10, 5, -5, 10]


def test_validate_pose():
    assert al.validate_pose([0] * 6) is None
    assert al.validate_pose([0] * 5) is not None


def test_build_poses_override_and_problems():
    poses, problems = al.build_poses(
        {"stow": [1, 2, 3, 4, 5, 6], "bogus": [0] * 6, "grasp": [1]})
    assert poses["stow"] == [1, 2, 3, 4, 5, 6]
    assert len(problems) == 2
    assert poses["grasp"] == al.DEFAULT_POSES["grasp"]


def test_interpolate_ends_at_target_and_respects_step():
    start = [0.0] * 6
    end = [10.0, 0, 0, 0, 0, -4.0]
    out = list(al.interpolate(start, end, 3.0))
    assert out[-1] == end
    assert len(out) == 4
    prev = start
    for p in out:
        assert max(abs(a - b) for a, b in zip(p, prev)) <= 3.0 + 1e-9
        prev = p


def test_interpolate_no_move_and_bad_step():
    assert list(al.interpolate([1] * 6, [1] * 6, 2.0)) == [[1] * 6]
    assert list(al.interpolate([0] * 6, [5] * 6, 0)) == [[5] * 6]


def test_plan_pick_gripper_overrides():
    poses, _ = al.build_poses({})
    seq = al.plan_sequence("pick", poses, 50.0, 0.0)
    assert len(seq) == len(al.SEQUENCES["pick"])
    assert seq[-1][-1] == 0.0
    assert seq[2][-1] == 0.0


def test_plan_deposit_opens_gripper():
    poses, _ = al.build_poses({})
    seq = al.plan_sequence("deposit", poses, 50.0, 0.0)
    assert seq[1][-1] == 50.0


def test_plan_unknown_sequence():
    with pytest.raises(KeyError):
        al.plan_sequence("dance", {}, 1, 0)


def test_grasp_held():
    assert al.grasp_held(8.0, 0.0, 5.0)
    assert not al.grasp_held(2.0, 0.0, 5.0)
