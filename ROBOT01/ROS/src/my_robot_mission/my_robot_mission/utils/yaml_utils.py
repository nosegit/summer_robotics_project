"""Mission YAML loading, normalization, and validation."""

from dataclasses import dataclass
import math
from pathlib import Path
import re

import yaml


COMMAND_PATTERN = re.compile(
    r'^(set_initial_pose|navigate_to_pose|navigate_to_drone_parcel|'
    r'acquire_parcel|search_parcel|servo_parcel|rotate_blind|idle|'
    r'open_gripper|pickup|open_door|close_door|wait|led_blink|'
    r'led_red|led_green|led_off)'
    r'(?:\(\s*([A-Za-z_][A-Za-z0-9_-]*|-?\d+)\s*\))?$'
)


@dataclass(frozen=True)
class MissionCommand:
    text: str
    name: str
    argument: object = None


@dataclass(frozen=True)
class DroneParcelConfig:
    topic: str
    observation_timeout: float
    minimum_confidence: float
    sample_count: int
    zone_boundary_y: float
    boundary_margin: float
    upper_staging_pose: object
    lower_staging_pose: object


@dataclass(frozen=True)
class MissionConfig:
    configured: bool
    start_pose: tuple
    poses: dict
    commands: tuple
    pickup_rotation: float
    initial_pose_publish_count: int
    initial_pose_publish_period: float
    localization_settle_time: float
    server_timeout: float
    map_frame: str
    doors: dict
    drone_parcel: DroneParcelConfig | None


def load_mission_file(path):
    with Path(path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('Mission file must contain a YAML mapping')
    return parse_mission_config(config)


def normalize_pose_key(value):
    text = str(value).strip()
    if re.fullmatch(r'-?\d+', text):
        return int(text)
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*', text):
        raise ValueError(
            f'Invalid pose name {text!r}; use letters, numbers, _ or -'
        )
    return text


def _pose(value, name):
    if not isinstance(value, (list, tuple)):
        raise ValueError(f'{name} must be [x, y, yaw]')
    values = list(value)
    if len(values) != 3 or not all(
        math.isfinite(float(item)) for item in values
    ):
        raise ValueError(f'{name} must contain three finite values: [x, y, yaw]')
    return tuple(float(item) for item in values)


def _variables(config):
    variables = {}
    if 'parcel_id' in config:
        try:
            parcel_id = int(config['parcel_id'])
        except (TypeError, ValueError) as error:
            raise ValueError('parcel_id must be a non-negative integer') from error
        if parcel_id < 0:
            raise ValueError('parcel_id must be a non-negative integer')
        variables['parcel_id'] = parcel_id
    return variables


def _commands(raw_commands, variables, poses):
    if not isinstance(raw_commands, list) or not raw_commands:
        raise ValueError('commands must be a non-empty YAML list')
    no_argument = {
        'set_initial_pose', 'rotate_blind', 'idle', 'open_gripper',
        'pickup', 'led_blink', 'led_red', 'led_green', 'led_off',
    }
    required_argument = {
        'navigate_to_pose', 'navigate_to_drone_parcel', 'acquire_parcel',
        'search_parcel', 'servo_parcel', 'open_door', 'close_door',
    }
    parsed = []
    for raw_command in raw_commands:
        text = str(raw_command).strip()
        match = COMMAND_PATTERN.fullmatch(text)
        if match is None:
            raise ValueError(f'Invalid mission command: {text!r}')
        name, argument_text = match.groups()
        if name in no_argument and argument_text is not None:
            raise ValueError(f'{name} does not accept an argument')
        if name in required_argument and argument_text is None:
            expected = (
                'pose name or ID' if name == 'navigate_to_pose'
                else 'integer or configured variable'
            )
            raise ValueError(f'{name} requires a {expected}')
        if name == 'wait' and argument_text is None:
            raise ValueError('wait requires a duration in seconds')
        argument = None
        if argument_text is not None:
            if name == 'navigate_to_pose':
                argument = normalize_pose_key(argument_text)
            else:
                try:
                    argument = int(argument_text)
                except ValueError as error:
                    if argument_text not in variables:
                        raise ValueError(
                            f'{name} requires an integer or configured '
                            f'variable, got {argument_text!r}'
                        ) from error
                    argument = variables[argument_text]
        if name == 'wait' and argument < 0:
            raise ValueError('wait duration must be non-negative')
        parsed.append(MissionCommand(text, name, argument))
    missing = {
        command.argument for command in parsed
        if command.name == 'navigate_to_pose' and command.argument not in poses
    }
    if missing:
        names = sorted(repr(value) for value in missing)
        raise ValueError(f'Commands reference undefined poses: {names}')
    return tuple(parsed)


def _doors(raw_doors, commands):
    if not isinstance(raw_doors, dict):
        raise ValueError('doors must be a YAML mapping of door IDs')
    doors = {}
    for door_id, value in raw_doors.items():
        if not isinstance(value, dict) or not value.get('host'):
            raise ValueError(f'doors[{door_id}] must contain a host')
        port = int(value.get('port', 50000))
        if not 1 <= port <= 65535:
            raise ValueError(f'doors[{door_id}].port must be between 1 and 65535')
        doors[int(door_id)] = (str(value['host']), port)
    missing = sorted({
        command.argument for command in commands
        if command.name in {'open_door', 'close_door'}
        and command.argument not in doors
    })
    if missing:
        raise ValueError(f'Commands reference undefined door IDs: {missing}')
    return doors


def _drone(raw, commands, poses):
    if not any(
        command.name == 'navigate_to_drone_parcel' for command in commands
    ):
        return None
    if not isinstance(raw, dict):
        raise ValueError(
            'drone_parcel configuration is required by '
            'navigate_to_drone_parcel'
        )
    result = DroneParcelConfig(
        topic=str(raw.get('topic', '/drone/parcel_detections')),
        observation_timeout=float(raw.get('observation_timeout', 3.0)),
        minimum_confidence=float(raw.get('minimum_confidence', 0.7)),
        sample_count=int(raw.get('sample_count', 5)),
        zone_boundary_y=float(raw.get('zone_boundary_y', -4.0)),
        boundary_margin=float(raw.get('boundary_margin', 0.3)),
        upper_staging_pose=normalize_pose_key(
            raw.get('upper_staging_pose', 'parcel_left')
        ),
        lower_staging_pose=normalize_pose_key(
            raw.get('lower_staging_pose', 'parcel_right')
        ),
    )
    if result.observation_timeout <= 0.0:
        raise ValueError('drone_parcel.observation_timeout must be positive')
    if not 0.0 <= result.minimum_confidence <= 1.0:
        raise ValueError('drone_parcel.minimum_confidence must be between 0 and 1')
    if result.sample_count < 1:
        raise ValueError('drone_parcel.sample_count must be at least 1')
    if result.boundary_margin < 0.0:
        raise ValueError('drone_parcel.boundary_margin cannot be negative')
    missing = [
        pose for pose in (result.upper_staging_pose, result.lower_staging_pose)
        if pose not in poses
    ]
    if missing:
        raise ValueError(f'drone_parcel references undefined staging poses: {missing}')
    return result


def parse_mission_config(config):
    variables = _variables(config)
    raw_poses = config.get('poses', {})
    if not isinstance(raw_poses, dict):
        raise ValueError('poses must be a YAML mapping of name or ID to [x, y, yaw]')
    poses = {
        normalize_pose_key(key): _pose(value, f'poses[{key}]')
        for key, value in raw_poses.items()
    }
    commands = _commands(config.get('commands'), variables, poses)
    return MissionConfig(
        configured=bool(config.get('mission_configured', False)),
        start_pose=_pose(config.get('start_pose'), 'start_pose'),
        poses=poses,
        commands=commands,
        pickup_rotation=float(config.get('pickup_rotation', 1.5708)),
        initial_pose_publish_count=int(config.get('initial_pose_publish_count', 5)),
        initial_pose_publish_period=float(config.get('initial_pose_publish_period', 0.5)),
        localization_settle_time=float(config.get('localization_settle_time', 2.0)),
        server_timeout=float(config.get('server_timeout', 60.0)),
        map_frame=str(config.get('map_frame', 'map')),
        doors=_doors(config.get('doors', {}), commands),
        drone_parcel=_drone(config.get('drone_parcel'), commands, poses),
    )
