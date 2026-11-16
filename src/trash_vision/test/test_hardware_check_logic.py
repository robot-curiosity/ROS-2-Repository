import copy
import os

import yaml

from trash_vision import hardware_check as hc
from trash_vision import hardware_check_logic as hl

PKG = os.path.join(os.path.dirname(__file__), "..")


def shipped():
    with open(os.path.join(PKG, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def drive_ok():
    cfg = shipped()
    p = cfg["drive"]["ros__parameters"]
    p.update(pwm_pins=[12, 13], dir_pins=[5, 6], enc_a_pins=[17, 22],
             enc_b_pins=[27, 23], ticks_per_rev=1300.0,
             wheel_radius_m=0.06, track_width_m=0.48)
    return cfg


def sonar_ok():
    cfg = shipped()
    p = cfg["ultrasonic"]["ros__parameters"]
    p["trig_pins"] = [4, 7, 8, 9, 10, 11, 16, 18]
    p["echo_pins"] = [19, 20, 21, 24, 25, 26, 2, 3]
    return cfg


def test_shipped_config_skips_unassembled_parts():
    res = hl.run_checks(shipped())
    assert res["drive"]["status"] == hl.SKIP
    assert res["sonar"]["status"] == hl.SKIP
    assert res["arm"]["status"] == hl.SKIP
    assert res["imu_gps"]["status"] == hl.SKIP
    assert hl.exit_code(res) == 0


def test_drive_pass_when_filled():
    assert hl.check_drive(drive_ok())["status"] == hl.PASS


def test_drive_partial_fails_and_names_unset_values():
    cfg = drive_ok()
    cfg["drive"]["ros__parameters"]["ticks_per_rev"] = 0.0
    res = hl.check_drive(cfg)
    assert res["status"] == hl.FAIL
    assert "unset: drive.ticks_per_rev" in res["errors"]


def test_drive_duplicate_pin_fails():
    cfg = drive_ok()
    cfg["drive"]["ros__parameters"]["dir_pins"] = [12, 6]
    res = hl.check_drive(cfg)
    assert res["status"] == hl.FAIL
    assert any("pin 12 is used more than once" in e for e in res["errors"])


def test_pin_outside_pi5_header_fails():
    cfg = drive_ok()
    cfg["drive"]["ros__parameters"]["pwm_pins"] = [12, 40]
    res = hl.check_drive(cfg)
    assert any("not a usable Pi 5 header GPIO" in e for e in res["errors"])
    cfg["drive"]["ros__parameters"]["pwm_pins"] = [1, 13]
    assert hl.check_drive(cfg)["status"] == hl.FAIL


def test_drive_wrong_length_fails():
    cfg = drive_ok()
    cfg["drive"]["ros__parameters"]["enc_a_pins"] = [17]
    res = hl.check_drive(cfg)
    assert res["status"] == hl.FAIL


def test_drive_wheel_radius_in_wrong_units_fails():
    cfg = drive_ok()
    cfg["drive"]["ros__parameters"]["wheel_radius_m"] = 60.0
    assert hl.check_drive(cfg)["status"] == hl.FAIL


def test_sonar_pass():
    assert hl.check_sonar(sonar_ok())["status"] == hl.PASS


def test_sonar_pin_shared_with_drive_is_reported_for_both():
    cfg = sonar_ok()
    cfg.update({"drive": drive_ok()["drive"]})
    cfg["ultrasonic"]["ros__parameters"]["trig_pins"][0] = 12
    sonar = hl.check_sonar(cfg)
    drive = hl.check_drive(cfg)
    assert sonar["status"] == hl.FAIL
    assert drive["status"] == hl.FAIL


def test_sonar_trig_equals_echo_conflict():
    cfg = sonar_ok()
    cfg["ultrasonic"]["ros__parameters"]["echo_pins"][1] = 7
    assert hl.check_sonar(cfg)["status"] == hl.FAIL


def test_sonar_half_set_sensor_fails_unused_sensor_is_ok():
    cfg = sonar_ok()
    p = cfg["ultrasonic"]["ros__parameters"]
    p["trig_pins"][7] = -1
    assert hl.check_sonar(cfg)["status"] == hl.FAIL
    p["echo_pins"][7] = -1
    res = hl.check_sonar(cfg)
    assert res["status"] == hl.PASS
    assert any("right_2 unused" in n for n in res["notes"])


def test_sonar_wrong_count_fails():
    cfg = sonar_ok()
    cfg["ultrasonic"]["ros__parameters"]["trig_pins"] = [4, 7]
    assert hl.check_sonar(cfg)["status"] == hl.FAIL


def test_arm_checks():
    cfg = shipped()
    assert hl.check_arm(cfg)["status"] == hl.SKIP
    cfg["arm"]["ros__parameters"]["port"] = "/dev/ttyACM0"
    assert hl.check_arm(cfg)["status"] == hl.PASS
    cfg["arm"]["ros__parameters"]["port"] = "ttyACM0"
    assert hl.check_arm(cfg)["status"] == hl.FAIL
    cfg["arm"]["ros__parameters"]["port"] = "/dev/ttyACM0"
    cfg["arm"]["ros__parameters"]["joint_min"][0] = 200.0
    assert hl.check_arm(cfg)["status"] == hl.FAIL


def test_camera_checks():
    cfg = shipped()
    assert hl.check_camera(cfg)["status"] == hl.PASS
    bad = copy.deepcopy(cfg)
    bad["detector"]["ros__parameters"]["camera_source"] = ""
    assert hl.check_camera(bad)["status"] == hl.FAIL
    bad = copy.deepcopy(cfg)
    bad["detector"]["ros__parameters"]["camera_pitch_deg"] = 120.0
    assert hl.check_camera(bad)["status"] == hl.FAIL


def test_imu_gps_checks_and_uart_collision():
    cfg = shipped()
    p = cfg["gps_imu"]["ros__parameters"]
    p.update(gps_port="/dev/ttyAMA0", imu_model="bno055")
    assert hl.check_imu_gps(cfg)["status"] == hl.PASS
    cfg["drive"] = drive_ok()["drive"]
    cfg["drive"]["ros__parameters"]["pwm_pins"] = [14, 13]
    res = hl.check_imu_gps(cfg)
    assert res["status"] == hl.FAIL
    assert any("GPS UART" in e for e in res["errors"])
    cfg["drive"]["ros__parameters"]["pwm_pins"] = [2, 13]
    assert any("I2C" in e for e in hl.check_imu_gps(cfg)["errors"])


def test_imu_gps_half_set_fails():
    cfg = shipped()
    cfg["gps_imu"]["ros__parameters"]["gps_port"] = "/dev/ttyAMA0"
    res = hl.check_imu_gps(cfg)
    assert res["status"] == hl.FAIL
    assert "unset: gps_imu.imu_model" in res["errors"]


def test_run_checks_unknown_subsystem_raises():
    try:
        hl.run_checks(shipped(), ["wings"])
    except ValueError as exc:
        assert "wings" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_report_and_exit_code():
    cfg = drive_ok()
    cfg["drive"]["ros__parameters"]["kp"] = 0.5
    cfg["drive"]["ros__parameters"]["dir_pins"] = [12, 6]
    res = hl.run_checks(cfg, ["drive", "arm"])
    text = hl.format_report(res)
    assert "[FAIL] drive" in text and "[SKIP] arm" in text
    assert hl.exit_code(res) == 1


def test_cli_main_on_shipped_yaml(capsys):
    path = os.path.join(PKG, "config", "robot.yaml")
    assert hc.main(["--config", path, "sonar"]) == 0
    assert "[SKIP] sonar" in capsys.readouterr().out
    assert hc.main(["--config", path, "wings"]) == 2
