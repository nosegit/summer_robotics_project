"""Read a drone result file and select a fixed parcel staging pose."""

import json
import math
from pathlib import Path


class DroneParcelSelector:
    def __init__(self, node, config):
        self.node = node
        self.config = config

    def select_staging_pose(self, marker_id):
        path = Path(self.config.result_file).expanduser()
        try:
            with path.open(encoding='utf-8') as stream:
                result = json.load(stream)
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f'Cannot read drone result file {path}: {error}'
            ) from error
        if not isinstance(result, dict):
            raise RuntimeError(f'Drone result file {path} must contain an object')
        try:
            x = float(result['x'])
            y = float(result['y'])
            radius = float(result['radius'])
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(
                f'Drone result file {path} requires numeric x, y, and radius'
            ) from error
        if not all(math.isfinite(value) for value in (x, y, radius)):
            raise RuntimeError(f'Drone result file {path} contains non-finite values')
        result_marker_id = result.get('marker_id')
        if result_marker_id is not None:
            try:
                result_marker_id = int(result_marker_id)
            except (TypeError, ValueError) as error:
                raise RuntimeError(
                    f'Drone result file {path} has an invalid marker_id'
                ) from error
            if result_marker_id != marker_id:
                raise RuntimeError(
                    f'Drone result is for marker {result_marker_id}, not {marker_id}'
                )

        map_x = self.config.map_origin_x + x * self.config.coordinate_scale
        map_y = self.config.map_origin_y + y * self.config.coordinate_scale
        upper = self.config.zone_boundary_y + self.config.boundary_margin
        lower = self.config.zone_boundary_y - self.config.boundary_margin
        if map_y >= upper:
            pose_key = self.config.upper_staging_pose
        elif map_y <= lower:
            pose_key = self.config.lower_staging_pose
        else:
            raise RuntimeError(
                f'Drone result map y={map_y:.3f} is inside the boundary margin'
            )
        self.node.get_logger().info(
            f'Loaded drone marker {marker_id} from {path}: '
            f'map position=({map_x:.3f}, {map_y:.3f}), '
            f'uncertainty radius={radius * self.config.coordinate_scale:.3f} m'
        )
        return pose_key, map_y
