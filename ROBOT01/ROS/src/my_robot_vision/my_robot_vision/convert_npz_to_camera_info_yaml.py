"""Convert an OpenCV .npz calibration archive into ROS camera-info YAML."""

import argparse
from pathlib import Path

import numpy as np
import yaml


MATRIX_KEYS = ('camera_matrix', 'cameraMatrix', 'mtx', 'K')
DISTORTION_KEYS = (
    'distortion_coefficients', 'dist_coeffs', 'distCoeffs', 'dist', 'D',
)


def _npz_value(calibration, keys, description, calibration_path):
    for key in keys:
        if key in calibration:
            return calibration[key]
    raise KeyError(
        f'{calibration_path} has no {description}. Expected one of: {", ".join(keys)}'
    )


def _matrix_entry(rows, cols, values):
    return {'rows': rows, 'cols': cols, 'data': values}


def convert_calibration(npz_path, output_path, width, height, camera_name, distortion_model):
    """Write a ROS camera-calibration YAML file from an OpenCV NPZ archive."""
    with np.load(npz_path, allow_pickle=False) as calibration:
        camera_matrix = _npz_value(
            calibration, MATRIX_KEYS, 'camera matrix', npz_path,
        )
        distortion = _npz_value(
            calibration, DISTORTION_KEYS, 'distortion coefficients', npz_path,
        )

    if camera_matrix.shape != (3, 3):
        raise ValueError(
            f'Camera matrix in {npz_path} must have shape (3, 3), got {camera_matrix.shape}'
        )

    camera_matrix_data = camera_matrix.astype(float).reshape(-1).tolist()
    distortion_data = distortion.astype(float).reshape(-1).tolist()
    projection = [
        camera_matrix_data[0], 0.0, camera_matrix_data[2], 0.0,
        0.0, camera_matrix_data[4], camera_matrix_data[5], 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    calibration_yaml = {
        'image_width': width,
        'image_height': height,
        'camera_name': camera_name,
        'camera_matrix': _matrix_entry(3, 3, camera_matrix_data),
        'distortion_model': distortion_model,
        'distortion_coefficients': _matrix_entry(1, len(distortion_data), distortion_data),
        'rectification_matrix': _matrix_entry(
            3, 3, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        ),
        'projection_matrix': _matrix_entry(3, 4, projection),
    }
    with output_path.open('w', encoding='utf-8') as output_file:
        yaml.safe_dump(calibration_yaml, output_file, sort_keys=False)


def main():
    parser = argparse.ArgumentParser(
        description='Convert an OpenCV .npz camera calibration to ROS YAML.'
    )
    parser.add_argument('input_npz', type=Path, help='Input OpenCV .npz calibration file.')
    parser.add_argument('output_yaml', type=Path, help='Output ROS camera-calibration YAML.')
    parser.add_argument('--width', type=int, required=True, help='Calibration image width in pixels.')
    parser.add_argument('--height', type=int, required=True, help='Calibration image height in pixels.')
    parser.add_argument('--camera-name', default='pi_camera_v2')
    parser.add_argument('--distortion-model', default='plumb_bob')
    args = parser.parse_args()

    if args.width <= 0 or args.height <= 0:
        parser.error('--width and --height must both be greater than zero')
    if not args.input_npz.is_file():
        parser.error(f'Input calibration file not found: {args.input_npz}')

    convert_calibration(
        args.input_npz, args.output_yaml, args.width, args.height,
        args.camera_name, args.distortion_model,
    )
    print(f'Wrote ROS camera calibration: {args.output_yaml}')


if __name__ == '__main__':
    main()
