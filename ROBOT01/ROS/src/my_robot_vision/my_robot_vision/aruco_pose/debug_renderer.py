import cv2
import numpy as np


class DebugRenderer:
    @staticmethod
    def _draw_axes(image, camera_matrix, distortion, rvec, tvec, axis_length):
        points = np.array([
            [0.0, 0.0, 0.0],
            [axis_length, 0.0, 0.0],
            [0.0, axis_length, 0.0],
            [0.0, 0.0, axis_length],
        ], dtype=np.float64)
        projected, _ = cv2.projectPoints(
            points, rvec, tvec, camera_matrix, distortion
        )
        pixels = np.rint(projected.reshape(-1, 2)).astype(int)
        origin = tuple(pixels[0])
        for endpoint, intensity in zip(pixels[1:], (255, 180, 100)):
            cv2.line(
                image, origin, tuple(endpoint), intensity, 2, cv2.LINE_AA
            )

    @staticmethod
    def draw_marker(image, camera_matrix, distortion, pose, axis_length):
        DebugRenderer._draw_axes(
            image, camera_matrix, distortion,
            pose.rvec, pose.tvec, axis_length,
        )

    @staticmethod
    def draw_cube(image, camera_matrix, distortion, estimate, axis_length):
        center_tvec = estimate.center.reshape((3, 1)).astype(np.float64)
        rvec, _ = cv2.Rodrigues(estimate.rotation)
        DebugRenderer._draw_axes(
            image, camera_matrix, distortion,
            rvec, center_tvec, axis_length,
        )
        projected, _ = cv2.projectPoints(
            np.zeros((1, 3), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64), center_tvec,
            camera_matrix, distortion,
        )
        pixel = tuple(np.rint(projected[0, 0]).astype(int))
        cv2.circle(image, pixel, 5, 255, -1)
        cv2.putText(
            image, f'cube {estimate.marker_id}',
            (pixel[0] + 7, pixel[1] - 7),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1,
            cv2.LINE_AA,
        )
