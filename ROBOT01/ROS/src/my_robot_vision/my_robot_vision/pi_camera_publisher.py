"""Publish Raspberry Pi Camera Module frames using the standard ROS camera API."""

import math
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class PiCameraPublisher(Node):
    """Capture Pi Camera 2 images with Picamera2 and publish image plus CameraInfo."""

    def __init__(self):
        super().__init__('pi_camera_publisher')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.frame_id = self.get_parameter('frame_id').value
        fps = self.get_parameter('fps').value
        if fps <= 0.0:
            raise ValueError('fps must be greater than zero')

        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise RuntimeError(
                'Picamera2 is required on the Raspberry Pi. Install python3-picamera2 '
                'from Raspberry Pi OS before launching this node.'
            ) from error

        self.camera = Picamera2()
        configuration = self.camera.create_video_configuration(
            main={'size': (self.width, self.height), 'format': 'RGB888'},
            controls={'FrameRate': float(fps)},
        )
        self.camera.configure(configuration)
        self.camera.start()

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.camera_info = self._load_camera_info(
            self.get_parameter('camera_info_url').value
        )
        self.timer = self.create_timer(1.0 / fps, self._publish_frame)

    def _load_camera_info(self, camera_info_url):
        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        if not camera_info_url:
            # A usable image stream, but not a calibration suitable for metric poses.
            focal_length = self.width / (2.0 * math.tan(1.0856 / 2.0))
            info.k = [focal_length, 0.0, self.width / 2.0, 0.0, focal_length,
                      self.height / 2.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [focal_length, 0.0, self.width / 2.0, 0.0, 0.0,
                      focal_length, self.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.get_logger().warning(
                'No camera_info_url supplied; using approximate intrinsics. '
                'Calibrate the physical camera before ArUco pose estimation.'
            )
            return info

        calibration_path = Path(camera_info_url.replace('file://', ''))
        if not calibration_path.is_file():
            raise FileNotFoundError(f'Camera calibration file not found: {calibration_path}')

        try:
            import yaml
        except ImportError as error:
            raise RuntimeError('python3-yaml is required to load camera calibration files.') from error
        with calibration_path.open(encoding='utf-8') as calibration_file:
            calibration = yaml.safe_load(calibration_file)
        info.width = calibration['image_width']
        info.height = calibration['image_height']
        info.distortion_model = calibration['distortion_model']
        info.d = calibration['distortion_coefficients']['data']
        info.k = calibration['camera_matrix']['data']
        info.r = calibration['rectification_matrix']['data']
        info.p = calibration['projection_matrix']['data']
        return info

    def _publish_frame(self):
        image = self.bridge.cv2_to_imgmsg(self.camera.capture_array('main'), encoding='rgb8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = self.frame_id
        self.camera_info.header = image.header
        self.image_publisher.publish(image)
        self.info_publisher.publish(self.camera_info)

    def destroy_node(self):
        self.camera.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
