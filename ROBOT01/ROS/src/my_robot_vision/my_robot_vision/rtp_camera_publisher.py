"""Turn a low-latency H.264/RTP stream into ROS camera topics.

The CSI camera remains on the Raspberry Pi host, where libcamera/Picamera2 is
available.  The container only decodes the stream, so it never needs access to
the libcamera IPA stack or the Picamera2 Python package.
"""

import math
from pathlib import Path

import gi
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402


class RtpCameraPublisher(Node):
    """Receive H.264 RTP on UDP and publish the newest decoded frame only."""

    def __init__(self):
        super().__init__('rtp_camera_publisher')
        self.declare_parameter('port', 5600)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('rotate_180', False)

        self.port = self.get_parameter('port').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rotate_180 = bool(self.get_parameter('rotate_180').value)
        if not 1 <= self.port <= 65535:
            raise ValueError('port must be between 1 and 65535')

        self.bridge = CvBridge()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_publisher = self.create_publisher(
            Image, 'camera/image_raw', sensor_qos
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, 'camera/camera_info', sensor_qos
        )
        self.camera_info = self._load_camera_info(
            self.get_parameter('camera_info_url').value
        )

        Gst.init(None)
        
        # OPTIMIZED PIPELINE:
        # 1. udpsrc buffer-size increased to prevent packet drops if the OS scheduler blips
        # 2. videoconvert n-threads=2 parallelizes the CPU-heavy BGR color conversion
        # 3. avdec_h264 is used (Software). See note below if running this on the Pi!
        pipeline = (
            f'udpsrc port={self.port} buffer-size=524288 '
            'caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" ! '
            'rtpjitterbuffer latency=0 drop-on-latency=true ! '
            # Frame-threaded H.264 decoding improves throughput by buffering
            # frames. One decoder thread is lower latency for this 640x480 feed.
            'rtph264depay ! h264parse ! avdec_h264 max-threads=1 ! '
            'queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! '
            'videoconvert n-threads=2 ! video/x-raw,format=BGR ! '
            'appsink name=framesink emit-signals=true sync=false max-buffers=1 drop=true'
        )
        
        self.pipeline = Gst.parse_launch(pipeline)
        self.appsink = self.pipeline.get_by_name('framesink')
        self.appsink.connect('new-sample', self._on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f'Listening for H.264/RTP camera frames on UDP {self.port}')

    def _load_camera_info(self, camera_info_url):
        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        if not camera_info_url:
            focal_length = self.width / (2.0 * math.tan(1.0856 / 2.0))
            info.k = [focal_length, 0.0, self.width / 2.0, 0.0, focal_length,
                      self.height / 2.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [focal_length, 0.0, self.width / 2.0, 0.0, 0.0,
                      focal_length, self.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.get_logger().warning(
                'No camera_info_url supplied; using approximate intrinsics. '
                'Calibrate the physical camera before metric pose estimation.'
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

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        buffer = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        width = caps.get_value('width')
        height = caps.get_value('height')
        success, mapped = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        try:
            # .copy() is vital here so we can unmap the GStreamer buffer immediately
            image_array = np.frombuffer(mapped.data, dtype=np.uint8).reshape((height, width, 3))
            if self.rotate_180:
                # The physical camera is mounted upside down. Flipping both
                # image axes is exactly a 180-degree image-plane rotation.
                image_array = image_array[::-1, ::-1]
            image = self.bridge.cv2_to_imgmsg(image_array.copy(), encoding='bgr8')
            image.header.stamp = self.get_clock().now().to_msg()
            image.header.frame_id = self.frame_id
            self.camera_info.header = image.header
            self.image_publisher.publish(image)
            self.info_publisher.publish(self.camera_info)
        finally:
            buffer.unmap(mapped)
        return Gst.FlowReturn.OK

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtpCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
