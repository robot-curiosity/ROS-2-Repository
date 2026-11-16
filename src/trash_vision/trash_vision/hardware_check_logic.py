"""Pure-Python bring-up checks for config/robot.yaml (no ROS needed).

Each subsystem gets PASS, FAIL or SKIP:
  SKIP  nothing is filled in yet (the part is not assembled)
  FAIL  something is unset, invalid or conflicting
  PASS  every required value is set and no pin problem was found
"""
from trash_vision import config_logic as cl

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"

# Usable BCM GPIO on the Pi 5 40-pin header. GPIO0/1 are the HAT ID EEPROM
# pins and are not for general use.
PI5_GPIO_MIN = 2
PI5_GPIO_MAX = 27
# Pins that other functions on the header claim (BCM numbers).
I2C1_PINS = (2, 3)
UART0_PINS = (14, 15)

SONAR_NAMES = ["front_1", "front_2", "rear_1", "rear_2",
               "left_1", "left_2", "right_1", "right_2"]

# (node, parameter) of every GPIO pin parameter, with a readable role.
PIN_PARAMS = {
    "drive": ["pwm_pins", "dir_pins", "enc_a_pins", "enc_b_pins"],
    "ultrasonic": ["trig_pins", "echo_pins"],
}
DRIVE_NAMES = ["left", "right"]


def _as_list(value):
    if isinstance(value, (list, tuple)):
        return list(value)
    return [] if value is None else [value]


def collect_pins(config):
    """List of (pin, label) for every set GPIO pin in the whole config."""
    used = []
    for node, params_names in PIN_PARAMS.items():
        params = cl.node_params(config, node)
        for name in params_names:
            names = SONAR_NAMES if node == "ultrasonic" else DRIVE_NAMES
            for idx, pin in enumerate(_as_list(params.get(name))):
                if isinstance(pin, int) and not isinstance(pin, bool) \
                        and pin >= 0:
                    who = names[idx] if idx < len(names) else str(idx)
                    used.append((pin, "%s.%s[%s]" % (node, name, who)))
    return used


def check_pin_range(pin_labels):
    """Errors for pins that are not usable Pi 5 header GPIO."""
    errors = []
    for pin, label in pin_labels:
        if pin < PI5_GPIO_MIN or pin > PI5_GPIO_MAX:
            errors.append(
                "%s = %d is not a usable Pi 5 header GPIO (valid BCM %d..%d)"
                % (label, pin, PI5_GPIO_MIN, PI5_GPIO_MAX))
    return errors


def check_conflicts(pin_labels, only_nodes=None):
    """Errors for pins used twice. Only conflicts that involve a pin of one
    of `only_nodes` are reported (all of them if None)."""
    by_pin = {}
    for pin, label in pin_labels:
        by_pin.setdefault(pin, []).append(label)
    errors = []
    for pin in sorted(by_pin):
        labels = by_pin[pin]
        if len(labels) < 2:
            continue
        if only_nodes is not None and not any(
                lab.split(".")[0] in only_nodes for lab in labels):
            continue
        errors.append("pin %d is used more than once: %s"
                      % (pin, ", ".join(labels)))
    return errors


def _result(status, errors, notes=None):
    return {"status": status, "errors": list(errors),
            "notes": list(notes or [])}


def _unset_errors(node, params):
    return ["unset: %s.%s" % (node, name)
            for name in cl.find_unset(node, params)]


def check_drive(config):
    params = cl.node_params(config, "drive")
    unset = cl.find_unset("drive", params)
    required = cl.REQUIRED["drive"]
    if len(unset) == len(required):
        return _result(SKIP, [], ["nothing filled in under drive:"])
    errors = _unset_errors("drive", params)
    for name in PIN_PARAMS["drive"]:
        if len(_as_list(params.get(name))) != 2:
            errors.append("drive.%s needs 2 entries [left, right]" % name)
    pins = [p for p in collect_pins(config) if p[1].startswith("drive.")]
    errors += check_pin_range(pins)
    errors += check_conflicts(collect_pins(config), {"drive"})
    notes = []
    wheel = params.get("wheel_radius_m")
    if isinstance(wheel, (int, float)) and wheel > 0.2:
        errors.append("drive.wheel_radius_m = %s is over 0.2 m; "
                      "is it in metres?" % wheel)
    if wheel and wheel > 0 and abs(wheel - 0.060) > 0.02:
        notes.append("wheel_radius_m differs from CAD 0.060 by > 2 cm")
    return _result(FAIL if errors else PASS, errors, notes)


def check_sonar(config):
    params = cl.node_params(config, "ultrasonic")
    trig = _as_list(params.get("trig_pins"))
    echo = _as_list(params.get("echo_pins"))
    errors = []
    if len(trig) != len(SONAR_NAMES) or len(echo) != len(SONAR_NAMES):
        errors.append("trig_pins and echo_pins need %d entries, order: %s"
                      % (len(SONAR_NAMES), ", ".join(SONAR_NAMES)))
        return _result(FAIL, errors)
    notes = []
    active = 0
    for name, t, e in zip(SONAR_NAMES, trig, echo):
        if t == -1 and e == -1:
            notes.append("%s unused (both pins -1)" % name)
        elif t == -1 or e == -1:
            errors.append("unset: %s has only one of trig/echo set" % name)
        else:
            active += 1
    if active == 0 and not errors:
        return _result(SKIP, [], ["no sonar pins filled in"])
    pins = [p for p in collect_pins(config)
            if p[1].startswith("ultrasonic.")]
    errors += check_pin_range(pins)
    errors += check_conflicts(collect_pins(config), {"ultrasonic"})
    notes.append("reminder: echo pins need a 5V->3.3V divider (wiring)")
    return _result(FAIL if errors else PASS, errors, notes)


def check_arm(config):
    params = cl.node_params(config, "arm")
    if cl.find_unset("arm", params):
        return _result(SKIP, [], ["arm.port not set"])
    errors = []
    if not str(params.get("port")).startswith("/dev/"):
        errors.append("arm.port should look like /dev/ttyACM0")
    lo = _as_list(params.get("joint_min"))
    hi = _as_list(params.get("joint_max"))
    if len(lo) != len(hi):
        errors.append("arm.joint_min and joint_max differ in length")
    elif any(a >= b for a, b in zip(lo, hi)):
        errors.append("arm.joint_min must be below joint_max for every joint")
    return _result(FAIL if errors else PASS, errors)


def check_camera(config):
    params = cl.node_params(config, "detector")
    errors = []
    for name in ("model_path", "camera_source"):
        if cl.is_unset(params.get(name)):
            errors.append("unset: detector.%s" % name)
    hfov = params.get("camera_hfov_deg", 0)
    if not 10 <= hfov <= 170:
        errors.append("detector.camera_hfov_deg = %s, expected 10..170"
                      % hfov)
    pitch = params.get("camera_pitch_deg", 0)
    if not 0 <= pitch <= 90:
        errors.append("detector.camera_pitch_deg = %s, expected 0..90"
                      % pitch)
    if cl.is_unset(params.get("camera_height_m")):
        errors.append("unset: detector.camera_height_m")
    return _result(FAIL if errors else PASS, errors)


def check_imu_gps(config):
    params = cl.node_params(config, "gps_imu")
    unset = cl.find_unset("gps_imu", params)
    if len(unset) == len(cl.REQUIRED["gps_imu"]):
        return _result(SKIP, [], ["gps_port and imu_model not set"])
    errors = _unset_errors("gps_imu", params)
    notes = []
    mine = collect_pins(config)
    for pin, label in mine:
        if pin in I2C1_PINS and params.get("imu_model"):
            errors.append("%s = %d collides with the IMU I2C bus (GPIO2/3)"
                          % (label, pin))
        if pin in UART0_PINS and params.get("gps_port"):
            errors.append("%s = %d collides with the GPS UART (GPIO14/15)"
                          % (label, pin))
    if params.get("imu_model") and params["imu_model"] != "bno055":
        errors.append("gps_imu.imu_model must be \"bno055\" (only backend)")
    if params.get("gps_baud", 0) <= 0:
        errors.append("gps_imu.gps_baud must be positive")
    return _result(FAIL if errors else PASS, errors, notes)


CHECKS = {
    "drive": check_drive,
    "sonar": check_sonar,
    "arm": check_arm,
    "camera": check_camera,
    "imu_gps": check_imu_gps,
}


def run_checks(config, subsystems=None):
    """{subsystem: result dict} for the named subsystems (default all)."""
    names = list(subsystems) if subsystems else list(CHECKS)
    unknown = [n for n in names if n not in CHECKS]
    if unknown:
        raise ValueError("unknown subsystem %s; choose from %s"
                         % (", ".join(unknown), ", ".join(CHECKS)))
    return {name: CHECKS[name](config or {}) for name in names}


def format_report(results):
    """Text report; one block per subsystem."""
    lines = []
    for name, res in results.items():
        lines.append("[%s] %s" % (res["status"], name))
        for err in res["errors"]:
            lines.append("    - %s" % err)
        for note in res["notes"]:
            lines.append("    . %s" % note)
    return "\n".join(lines)


def exit_code(results):
    """1 if any subsystem FAILed, else 0 (SKIP is not a failure)."""
    return 1 if any(r["status"] == FAIL for r in results.values()) else 0
