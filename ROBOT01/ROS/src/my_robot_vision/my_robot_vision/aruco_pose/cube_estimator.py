import time

import numpy as np

from .models import CubeEstimate


class CubeEstimator:
    """Fuse visible faces into stable per-ID cube poses."""

    def __init__(
        self, cube_size, vertical_threshold, switch_hysteresis,
        filter_enabled, filter_alpha, outlier_threshold,
        filter_reset_timeout,
    ):
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError('position_filter_alpha must be in the range (0, 1]')
        self.cube_size = cube_size
        self.vertical_threshold = vertical_threshold
        self.switch_hysteresis = switch_hysteresis
        self.filter_enabled = filter_enabled
        self.filter_alpha = filter_alpha
        self.outlier_threshold = outlier_threshold
        self.filter_reset_timeout = filter_reset_timeout
        self.previous_normals = {}
        self.filtered_centers = {}

    def face_candidate(self, marker_pose):
        center = (
            marker_pose.tvec.ravel()
            - marker_pose.normal * (self.cube_size / 2.0)
        )
        return center, marker_pose.normal

    def estimate(self, marker_id, faces):
        measured = np.median(np.array([face[0] for face in faces]), axis=0)
        center = self._filter_center(marker_id, measured)
        normal = self._select_face(marker_id, faces)[1].copy()
        previous = self.previous_normals.get(marker_id)
        if previous is not None and np.dot(normal, previous) < 0.0:
            normal = -normal
        self.previous_normals[marker_id] = normal
        return CubeEstimate(
            marker_id, center, normal,
            self._canonical_rotation(normal), len(faces),
        )

    def _filter_center(self, marker_id, measurement):
        if not self.filter_enabled:
            return measurement
        now = time.monotonic()
        previous = self.filtered_centers.get(marker_id)
        if previous is None or now - previous[2] > self.filter_reset_timeout:
            filtered = measurement.copy()
        else:
            previous_center, previous_measurement, _ = previous
            jump = np.linalg.norm(measurement - previous_measurement)
            if self.outlier_threshold > 0.0 and jump > self.outlier_threshold:
                # Hold the filtered output for one observation, but remember
                # the rejected measurement. If the next observation agrees
                # with it, the filter can recover instead of comparing against
                # the old pose and rejecting forever.
                self.filtered_centers[marker_id] = (
                    previous_center.copy(), measurement.copy(), now
                )
                return previous_center.copy()
            filtered = (
                self.filter_alpha * measurement
                + (1.0 - self.filter_alpha) * previous_center
            )
        self.filtered_centers[marker_id] = (
            filtered.copy(), measurement.copy(), now
        )
        return filtered

    def _select_face(self, marker_id, faces):
        horizontal = [
            face for face in faces
            if abs(float(face[1][1])) <= self.vertical_threshold
        ]
        selectable = horizontal if horizontal else faces

        def facing_score(face):
            view = face[0]
            view_norm = np.linalg.norm(view)
            if view_norm < 1e-9:
                return -1.0
            return -float(np.dot(face[1], view / view_norm))

        best = max(selectable, key=facing_score)
        previous = self.previous_normals.get(marker_id)
        if previous is None:
            return best
        same_face = max(
            selectable, key=lambda face: float(np.dot(face[1], previous))
        )
        similar = float(np.dot(same_face[1], previous)) >= 0.7
        competitive = (
            facing_score(same_face)
            >= facing_score(best) - self.switch_hysteresis
        )
        return same_face if similar and competitive else best

    @staticmethod
    def _canonical_rotation(normal):
        z_axis = normal / np.linalg.norm(normal)
        camera_up = np.array([0.0, -1.0, 0.0])
        y_axis = camera_up - np.dot(camera_up, z_axis) * z_axis
        if np.linalg.norm(y_axis) < 1e-6:
            camera_right = np.array([1.0, 0.0, 0.0])
            y_axis = camera_right - np.dot(camera_right, z_axis) * z_axis
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        return np.column_stack((x_axis, y_axis, z_axis))
