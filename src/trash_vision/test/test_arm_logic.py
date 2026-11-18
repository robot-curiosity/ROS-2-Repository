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


def test_plan_pick_poses_reachable_target():
    poses, why = al.plan_pick_poses(0.36, 0.0)
    assert why == "ok"
    assert set(poses) == {"pre_grasp", "grasp", "lift"}
    for pose in poses.values():
        assert len(pose) == len(al.JOINT_NAMES)
    assert poses["grasp"][5] == al.PickConfig().gripper_open
    assert poses["lift"][5] == 0.0
    assert poses["grasp"][:1] == pytest.approx([0.0], abs=0.5)


def test_plan_pick_poses_pan_follows_side():
    left, _ = al.plan_pick_poses(0.36, 0.08)
    right, _ = al.plan_pick_poses(0.36, -0.08)
    assert left is not None and right is not None
    assert left["grasp"][0] == pytest.approx(-right["grasp"][0], abs=1.0)
    assert abs(left["grasp"][0]) > 5.0


def test_plan_pick_poses_rejects_outside_zone():
    for x, y in [(0.2, 0.0), (0.8, 0.0), (0.4, 0.9)]:
        poses, why = al.plan_pick_poses(x, y)
        assert poses is None
        assert "pick zone" in why


def test_plan_pick_poses_rejects_unreachable():
    cfg = al.PickConfig(mount_x=-0.5)
    poses, why = al.plan_pick_poses(0.4, 0.0, cfg)
    assert poses is None
    assert "reach" in why


def test_plan_pick_poses_sign_offset_radians():
    base = al.PickConfig(use_degrees=False)
    flipped = al.PickConfig(use_degrees=False, joint_sign=(-1, 1, 1, 1),
                            joint_offset=(0.0, 0.5, 0.0, 0.0))
    a, _ = al.plan_pick_poses(0.36, 0.05, base)
    b, _ = al.plan_pick_poses(0.36, 0.05, flipped)
    assert b["grasp"][0] == pytest.approx(-a["grasp"][0])
    assert b["grasp"][1] == pytest.approx(a["grasp"][1] + 0.5)


def test_poses_usable_by_plan_sequence():
    poses, _ = al.plan_pick_poses(0.36, 0.0)
    merged = dict(al.DEFAULT_POSES)
    merged.update(poses)
    targets = al.plan_sequence("pick", merged, 50.0, 0.0)
    assert targets[1] == poses["grasp"]
