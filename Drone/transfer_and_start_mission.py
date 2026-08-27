"""Copy a completed drone result to the robot and optionally start its mission."""

import argparse
import json
from pathlib import Path
import subprocess
import sys


def validate_result(path):
    with path.open(encoding='utf-8') as stream:
        result = json.load(stream)
    if not isinstance(result, dict):
        raise ValueError('result must be a JSON object')
    missing = {'x', 'y', 'radius'} - result.keys()
    if missing:
        raise ValueError(f'result is missing: {", ".join(sorted(missing))}')
    for name in ('x', 'y', 'radius'):
        float(result[name])


def main():
    parser = argparse.ArgumentParser(
        description='Transfer the drone JSON result after changing to robot Wi-Fi.'
    )
    parser.add_argument('--robot', required=True, help='SSH target, e.g. pi@192.168.1.20')
    parser.add_argument(
        '--result',
        default=str(Path(__file__).with_name('targetLocation_robot.json')),
        help='Result to transfer (default: Drone/targetLocation_robot.json)',
    )
    parser.add_argument(
        '--remote-file', default='/tmp/targetLocation.json',
        help='Path read by my_robot_mission on the Raspberry Pi',
    )
    parser.add_argument(
        '--start-command',
        help='Optional command to run over SSH after a successful transfer',
    )
    args = parser.parse_args()

    result_path = Path(args.result).expanduser()
    try:
        validate_result(result_path)
        subprocess.run(
            ['scp', str(result_path), f'{args.robot}:{args.remote_file}'],
            check=True,
        )
        if args.start_command:
            subprocess.run(['ssh', args.robot, args.start_command], check=True)
    except (OSError, ValueError, json.JSONDecodeError, subprocess.CalledProcessError) as error:
        print(f'Transfer failed: {error}', file=sys.stderr)
        return 1
    print(f'Transferred {result_path} to {args.robot}:{args.remote_file}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
