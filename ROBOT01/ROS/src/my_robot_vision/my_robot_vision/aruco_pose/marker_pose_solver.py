import cv2
import numpy as np

from .models import MarkerPose


class MarkerPoseSolver:
    """Estimate a camera-facing pose for one planar marker."""

    def __init__(self, marker_size):
        half_size = marker_size / 2.0
        self.marker_edges = np.array([
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ], dtype=np.float32)

    def solve(self, marker_corners, camera_matrix, distortion):
        result = cv2.solvePnPGeneric(
            self.marker_edges, marker_corners, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not result[0]:
            return None
        errors = result[3] if len(result) > 3 else None
        candidates = []
        for index, (rvec, tvec) in enumerate(zip(result[1], result[2])):
            rotation, _ = cv2.Rodrigues(rvec)
            normal = rotation[:, 2]
            camera_facing = float(np.dot(normal, tvec.ravel())) < 0.0
            error = 0.0 if errors is None else float(
                np.asarray(errors[index]).reshape(-1)[0]
            )
            candidates.append(
                (not camera_facing, error, rvec, tvec, rotation, normal)
            )
        _, _, rvec, tvec, rotation, normal = min(
            candidates, key=lambda candidate: (candidate[0], candidate[1])
        )
        if np.dot(normal, tvec.ravel()) > 0.0:
            normal = -normal
        return MarkerPose(rvec, tvec, rotation, normal)
