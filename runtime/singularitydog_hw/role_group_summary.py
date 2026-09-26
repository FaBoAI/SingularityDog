"""Group an existing twelve-axis read-only capture by joint role.

This only reads a saved JSON file. It never opens CAN or authorizes motion.
Raw output-shaft angles are not calibrated robot joint angles.
"""

import argparse
import json
import math
from pathlib import Path


ROLES = (
    ("foot", "足先側", (1, 4, 7, 10)),
    ("thigh", "上脚", (2, 5, 8, 11)),
    ("hip", "付け根", (3, 6, 9, 12)),
)
LEGS = ("FR", "FL", "RR", "RL")


def summarize(snapshot):
    """Validate one complete capture and return three four-axis groups."""
    if (type(snapshot) is not dict or snapshot.get("status") != "READ_ONLY_COMPLETE"
            or snapshot.get("read_only") is not True
            or snapshot.get("motor_enable_sent") is not False
            or snapshot.get("errors") != []):
        raise ValueError("A complete no-enable read-only capture is required")
    motors = snapshot.get("motors")
    if type(motors) is not dict or set(motors) != {str(i) for i in range(1, 13)}:
        raise ValueError("Capture must contain exactly IDs 1..12")
    groups = []
    for role, label, ids in ROLES:
        rows = []
        for leg, mid in zip(LEGS, ids):
            motor = motors[str(mid)]
            if type(motor) is not dict or motor.get("uid_match") is not True:
                raise ValueError(f"ID{mid} identity is not verified")
            values = (motor.get("position_last_rad"), motor.get("position_range_rad"),
                      motor.get("current_max_abs_A"))
            voltages = motor.get("voltage_V")
            if (any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                    or values[1] < 0 or values[2] < 0
                    or type(voltages) is not list or not voltages
                    or any(type(v) not in (int, float) or not math.isfinite(v)
                           for v in voltages)):
                raise ValueError(f"ID{mid} has incomplete or nonfinite telemetry")
            rows.append({"leg": leg, "motor_id": mid,
                         "raw_position_rad": values[0],
                         "observed_position_range_deg": math.degrees(values[1]),
                         "max_abs_current_A": values[2],
                         "minimum_voltage_V": min(voltages)})
        groups.append({"role": role, "label": label, "motors": rows})
    return {"boot_id": snapshot.get("boot_id"), "status": "GROUPED_READ_ONLY_ONLY",
            "joint_calibration_verified": False, "powered_hold_verified": False,
            "motion_verified": False, "groups": groups}


def markdown(report):
    lines = ["|部位|右前 FR|左前 FL|右後 RR|左後 RL|最大位置変動|最低電圧|",
             "|---|---|---|---|---|---:|---:|"]
    for group in report["groups"]:
        rows = group["motors"]
        cells = [f"ID{row['motor_id']}" for row in rows]
        lines.append("|" + "|".join([group["label"], *cells,
            f"{max(row['observed_position_range_deg'] for row in rows):.3f}°",
            f"{min(row['minimum_voltage_V'] for row in rows):.2f} V"]) + "|")
    lines.append("\n位置変動は脱力・無駆動中の生角度。保持・校正・動作の合格判定ではありません。")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, help="Saved postboot read-only summary.json")
    args = parser.parse_args(argv)
    try:
        report = summarize(json.loads(args.capture.read_text()))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    print(markdown(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
