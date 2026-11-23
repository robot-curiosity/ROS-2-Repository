import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from trash_vision import arm_logic as al  # noqa: E402
from trash_vision import sim_logic as sl  # noqa: E402
from trash_vision import sim_mission as sm  # noqa: E402

SEEDS = (1, 2, 3)
# Software targets for the sim only, not robot results.
MIN_FINAL_RATE = 0.8      # fraction of litter eventually bagged, per seed
MIN_TYPE_RATE = 0.6       # per litter type, summed over the seeds


def _run(seed, litter=None, **kw):
    return sm.run_mission(litter or sm.TYPED_LITTER,
                          noise=sl.SimNoise(seed=seed), arm=al.PickConfig(),
                          **kw)


def test_pickup_rate_by_litter_type_with_noise_and_arm():
    got, total, tries, wins = {}, {}, {}, {}
    for seed in SEEDS:
        r = _run(seed)
        assert r['fraction'] >= MIN_FINAL_RATE
        for k, n in r['total_by_type'].items():
            total[k] = total.get(k, 0) + n
            got[k] = got.get(k, 0) + r['collected_by_type'].get(k, 0)
        for k, (a, s) in r['grasp_stats'].items():
            tries[k] = tries.get(k, 0) + a
            wins[k] = wins.get(k, 0) + s
    for kind in sorted(total):
        rate = got[kind] / total[kind]
        print('%-8s bagged %d/%d (%.0f%%), grasp %d/%d' % (
            kind, got[kind], total[kind], 100 * rate,
            wins.get(kind, 0), tries.get(kind, 0)))
        assert rate >= MIN_TYPE_RATE


def test_same_seed_is_repeatable_and_seeds_differ():
    a = _run(1, duration=120.0)
    b = _run(1, duration=120.0)
    assert a['collected'] == b['collected'] and a['states'] == b['states']
    assert a['grasp_stats'] == b['grasp_stats']
    w1 = sl.SimWorld(noise=sl.SimNoise(seed=1))
    w2 = sl.SimWorld(noise=sl.SimNoise(seed=2))
    assert w1.ranges() != w2.ranges()


def test_noise_off_is_ideal():
    w = sl.SimWorld(x=1.0, y=2.0)
    assert w.ranges() == w.ranges()
    assert w.ranges()['rear_2'] < 1.1


def test_pick_outside_pick_zone_fails_with_arm_model():
    # In front but 0.60 m away: past the 0.56 m zone limit.
    w = sl.SimWorld(litter=[(2.6, 2.0)], x=2.0, y=2.0, arm=al.PickConfig())
    ok, msg = w.pick()
    assert not ok and 'pick zone' in msg and len(w.litter) == 1


def test_pick_unreachable_by_arm_fails():
    # Inside the zone (0.50 m) but beyond the placeholder arm's reach.
    w = sl.SimWorld(litter=[(2.5, 2.0)], x=2.0, y=2.0, arm=al.PickConfig())
    ok, msg = w.pick()
    assert not ok and 'reachable' in msg


def test_pick_in_reach_succeeds_without_noise():
    w = sl.SimWorld(litter=[(2.38, 2.0, 'bottle')], x=2.0, y=2.0,
                    arm=al.PickConfig())
    ok, _ = w.pick()
    assert ok and w.held_type == 'bottle'
    w.deposit()
    assert w.collected_by_type == {'bottle': 1}


def test_grasp_probability_zero_never_picks_and_one_always_does():
    for prob, expect in ((0.0, False), (1.0, True)):
        nz = sl.SimNoise(seed=4, grasp_prob={'can': prob})
        w = sl.SimWorld(litter=[(2.38, 2.0)], x=2.0, y=2.0, noise=nz,
                        arm=al.PickConfig())
        assert w.pick()[0] is expect
        assert w.grasp_stats['can'] == [1, int(expect)]


def test_detection_dropout_and_false_positive_rates():
    nz = sl.SimNoise(seed=5, detect_dropout_prob=1.0, false_positive_prob=0.0)
    w = sl.SimWorld(litter=[(2.5, 2.0)], noise=nz)
    assert all(not w.target(i)['detected'] for i in range(20))
    nz = sl.SimNoise(seed=5, detect_dropout_prob=0.0, false_positive_prob=1.0)
    w = sl.SimWorld(litter=[], noise=nz)
    assert all(w.target(i)['detected'] for i in range(20))


def test_sonar_dropout_and_noise():
    nz = sl.SimNoise(seed=6, sonar_dropout_prob=1.0)
    w = sl.SimWorld(x=0.5, y=2.0, noise=nz)
    assert all(v == float('inf') for v in w.ranges().values())
    nz = sl.SimNoise(seed=6, sonar_dropout_prob=0.0, sonar_sigma=0.02)
    w = sl.SimWorld(x=0.5, y=2.0, noise=nz)
    clean = sl.SimWorld(x=0.5, y=2.0)
    diff = abs(w.ranges()['rear_2'] - clean.ranges()['rear_2'])
    assert 0.0 < diff < 0.2
