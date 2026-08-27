"""Filter drone detections and select a fixed parcel staging pose."""

from collections import defaultdict, deque
import statistics
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection3DArray


class DroneParcelSelector:
    def __init__(self, node, config, map_frame, mission_timeout):
        self.node = node
        self.config = config
        self.map_frame = map_frame
        self.mission_timeout = mission_timeout
        self.observations = defaultdict(deque)
        self.subscription = node.create_subscription(
            Detection3DArray, config.topic, self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message):
        if message.header.frame_id != self.map_frame:
            self.node.get_logger().warning(
                f'Ignoring drone detections in frame '
                f'{message.header.frame_id!r}; expected {self.map_frame!r}',
                throttle_duration_sec=5.0,
            )
            return
        received = time.monotonic()
        for detection in message.detections:
            for result in detection.results:
                try:
                    marker_id = int(result.hypothesis.class_id)
                except (TypeError, ValueError):
                    continue
                if float(result.hypothesis.score) < self.config.minimum_confidence:
                    continue
                samples = self.observations[marker_id]
                samples.append((float(result.pose.pose.position.y), received))
                while len(samples) > self.config.sample_count:
                    samples.popleft()

    def select_staging_pose(self, marker_id):
        deadline = time.monotonic() + self.mission_timeout
        median_y = None
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            samples = self.observations[marker_id]
            while samples and now - samples[0][1] > self.config.observation_timeout:
                samples.popleft()
            if len(samples) >= self.config.sample_count:
                median_y = statistics.median(item[0] for item in samples)
                upper = self.config.zone_boundary_y + self.config.boundary_margin
                lower = self.config.zone_boundary_y - self.config.boundary_margin
                if median_y >= upper:
                    return self.config.upper_staging_pose, median_y
                if median_y <= lower:
                    return self.config.lower_staging_pose, median_y
            rclpy.spin_once(self.node, timeout_sec=0.1)
        detail = (
            'no stable recent observation'
            if median_y is None
            else f'median y={median_y:.3f} is inside the boundary margin'
        )
        raise RuntimeError(
            f'Cannot select a parcel zone for marker {marker_id}: {detail}'
        )
