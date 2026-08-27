"""Convert a target position from the drone frame to the robot frame.

The drone's +x/+y axes point in the opposite directions to the robot's axes,
so the frame conversion is a 180-degree rotation followed by translation to
the robot's start position:

    robot_x = robot_start_x - drone_x / 100
    robot_y = robot_start_y - drone_y / 100

The input target is in centimetres. The output and robot start position are in
metres.
"""

import argparse
import json
import math
from pathlib import Path
import sys


def finite_number(value, name):
    """Return *value* as a finite float or raise a useful error."""
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("{} must be a number".format(name)) from error
    if not math.isfinite(number):
        raise ValueError("{} must be finite".format(name))
    return number


def convert_target(target, robot_start_x, robot_start_y):
    """Return a copy of *target* expressed in the robot frame."""
    if not isinstance(target, dict):
        raise ValueError("input JSON must contain an object")

    drone_x = finite_number(target.get("x"), "x")
    drone_y = finite_number(target.get("y"), "y")
    if "radius" in target:
        finite_number(target["radius"], "radius")

    converted = dict(target)
    converted["x"] = robot_start_x - drone_x / 100.0
    converted["y"] = robot_start_y - drone_y / 100.0
    if "radius" in target:
        converted["radius"] = float(target["radius"]) / 100.0
    return converted


def main():
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description=(
            "Convert a centimetre drone target to metres, rotate it by 180 "
            "degrees, and offset it by the robot start position."
        )
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=script_dir / "targetLocation.json",
        help="drone result JSON (default: Drone/targetLocation.json)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=script_dir / "targetLocation_robot.json",
        help="converted JSON (default: Drone/targetLocation_robot.json)",
    )
    parser.add_argument(
        "--start-x", type=float, default=-1.0,
        help="robot start-frame x coordinate in metres (default: -1)",
    )
    parser.add_argument(
        "--start-y", type=float, default=-6.0,
        help="robot start-frame y coordinate in metres (default: -6)",
    )
    args = parser.parse_args()

    try:
        start_x = finite_number(args.start_x, "start-x")
        start_y = finite_number(args.start_y, "start-y")
        with args.input.open(encoding="utf-8") as stream:
            target = json.load(stream)
        converted = convert_target(target, start_x, start_y)
        with args.output.open("w", encoding="utf-8") as stream:
            json.dump(converted, stream, indent=2)
            stream.write("\n")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print("Conversion failed: {}".format(error), file=sys.stderr)
        return 1

    print(
        "Converted {} -> {}: x={}, y={}".format(
            args.input, args.output, converted["x"], converted["y"]
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
