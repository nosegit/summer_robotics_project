import cv2
import numpy as np


class DebugRenderer:
    @staticmethod
    def draw_marker(image, camera_matrix, distortion, pose, axis_length):
        cv2.drawFrameAxes(
            image, camera_matrix, distortion,
            pose.rvec, pose.tvec, axis_length,
        )

    @staticmethod
    def draw_cube(image, camera_matrix, distortion, estimate, axis_length):
        center_tvec = estimate.center.reshape((3, 1)).astype(np.float64)
        rvec, _ = cv2.Rodrigues(estimate.rotation)
        cv2.drawFrameAxes(
            image, camera_matrix, distortion,
            rvec, center_tvec, axis_length,
        )
        projected, _ = cv2.projectPoints(
            np.zeros((1, 3), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64), center_tvec,
            camera_matrix, distortion,
        )
        pixel = tuple(np.rint(projected[0, 0]).astype(int))
        cv2.circle(image, pixel, 5, (0, 255, 255), -1)
        cv2.putText(
            image, f'cube {estimate.marker_id}',
            (pixel[0] + 7, pixel[1] - 7),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
            cv2.LINE_AA,
        )
