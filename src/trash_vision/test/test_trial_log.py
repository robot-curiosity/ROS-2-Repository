import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from trash_vision import behavior_logic as bl  # noqa: E402
from trash_vision import sim_mission as sm  # noqa: E402
from trash_vision import trial_log as tl  # noqa: E402

CAN = {"detected": True, "confidence": 0.9, "horizontal_error": 0.0,
       "floor_point_normalized": {"x": 0.5, "y": 0.95},
       "class_name": "can"}
CLEAR = [5.0, 5.0]


def drive(machine, rec, now, target, arm=None):
    if arm is not None:
        machine.arm_result(arm, now, {"PICK": "pick", "DEPOSIT": "deposit",
                                      "STOW": "stow"}[machine.state])
        rec.update(now, target)
    machine.step(now, target, CLEAR, CLEAR, CLEAR, CLEAR)
    rec.update(now, target)


def make():
    out = []
    m = bl.BehaviorMachine()
    return m, tl.TrialRecorder(m, out.append, "r1"), out


def test_full_success_is_one_record():
    m, rec, out = make()
    far = dict(CAN, floor_point_normalized={"x": 0.5, "y": 0.3})
    drive(m, rec, 0.0, far)
    assert m.state == bl.APPROACH
    drive(m, rec, 0.1, CAN)
    assert m.state == bl.PICK
    drive(m, rec, 0.2, CAN, arm=True)
    assert m.state == bl.DEPOSIT
    drive(m, rec, 0.3, None, arm=True)
    assert len(out) == 1
    r = out[0]
    assert r["approached"] and r["picked"] and r["success"]
    assert r["litter_type"] == "can" and r["failure_reason"] is None
    assert r["run"] == "r1" and r["t_end"] >= r["t_start"]


def test_target_lost_during_approach():
    m, rec, out = make()
    far = dict(CAN, floor_point_normalized={"x": 0.5, "y": 0.3})
    drive(m, rec, 0.0, far)
    drive(m, rec, 10.0, None)
    assert len(out) == 1
    assert not out[0]["approached"] and not out[0]["success"]
    assert out[0]["failure_reason"] == "target_lost"


def test_approach_timeout_reason():
    m, rec, out = make()
    far = dict(CAN, floor_point_normalized={"x": 0.5, "y": 0.3})
    t = 0.0
    while t < 40.0 and not out:
        drive(m, rec, t, far)
        t += 0.5
    assert out and out[0]["failure_reason"] == "approach_timeout"


def test_pick_failure_recorded_and_unknown_type():
    m, rec, out = make()
    can = {k: v for k, v in CAN.items() if k != "class_name"}
    drive(m, rec, 0.0, dict(can, floor_point_normalized={"x": .5, "y": .3}))
    drive(m, rec, 0.1, can)
    drive(m, rec, 0.2, can, arm=False)
    assert len(out) == 1
    assert out[0]["approached"] and not out[0]["picked"]
    assert out[0]["failure_reason"] == "pick_failed"
    assert out[0]["litter_type"] == "unknown"


def test_pick_arm_timeout_reason():
    m, rec, out = make()
    drive(m, rec, 0.0, dict(CAN, floor_point_normalized={"x": .5, "y": .3}))
    drive(m, rec, 0.1, CAN)
    drive(m, rec, 0.2, CAN)
    drive(m, rec, 100.0, None)
    assert out and out[0]["failure_reason"] == "arm_timeout"


def test_deposit_failure_recorded():
    m, rec, out = make()
    drive(m, rec, 0.0, dict(CAN, floor_point_normalized={"x": .5, "y": .3}))
    drive(m, rec, 0.1, CAN)
    drive(m, rec, 0.2, CAN, arm=True)
    drive(m, rec, 0.3, None, arm=False)
    assert len(out) == 1 and out[0]["picked"] and not out[0]["success"]
    assert out[0]["failure_reason"] == "deposit_failed"


def test_jsonl_roundtrip_and_damaged_lines(tmp_path):
    path = str(tmp_path / "sub" / "a.jsonl")
    w = tl.JsonlWriter(path)
    w({"run": "a", "success": True})
    w.close()
    with open(path, "a") as fh:
        fh.write("not json\n\n[1]\n")
    assert tl.read_records([path]) == [{"run": "a", "success": True}]


def _rec(run, typ, appr, pick, ok, why=None):
    return {"run": run, "litter_type": typ, "approached": appr,
            "picked": pick, "success": ok, "failure_reason": why}


def test_summary_by_type_and_run():
    recs = [_rec("r1", "can", 1, 1, 1), _rec("r1", "can", 1, 0, 0, "pick_failed"),
            _rec("r2", "bottle", 0, 0, 0, "target_lost"),
            _rec("r2", "bottle", 1, 1, 1)]
    s = tl.summarize(recs)
    assert s["overall"]["attempts"] == 4 and s["overall"]["rate"] == 0.5
    assert s["by_type"]["can"]["rate"] == 0.5
    assert s["by_type"]["bottle"]["failures"] == {"target_lost": 1}
    assert s["by_run"]["r1"]["picked"] == 1
    text = tl.format_summary(s)
    assert "can" in text and "r2" in text and "50%" in text


def test_summary_empty():
    s = tl.summarize([])
    assert s["overall"]["rate"] == 0.0 and s["by_type"] == {}


def test_find_logs_and_cli(tmp_path, capsys):
    from trash_vision import trial_summary as ts
    assert ts.main([str(tmp_path)]) == 1
    w = tl.JsonlWriter(str(tmp_path / "x.jsonl"))
    w(_rec("x", "can", 1, 1, 1))
    w.close()
    assert tl.find_logs([str(tmp_path)]) == [str(tmp_path / "x.jsonl")]
    assert ts.main([str(tmp_path)]) == 0
    assert "100%" in capsys.readouterr().out


def test_sim_mission_logs_match_collected():
    out = []
    r = sm.run_mission(sm.OPEN_LITTER, recorder_sink=out.append)
    ok = sum(1 for a in out if a["success"])
    assert ok == r["collected"]
    assert all(json.dumps(a) for a in out)
