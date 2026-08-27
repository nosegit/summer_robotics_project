#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from my_robot_vision.msg import CubeDetection, CubeDetectionArray

from .cube_estimator import CubeEstimator
from .debug_renderer import DebugRenderer
from .marker_pose_solver import MarkerPoseSolver


class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')
        defaults = {
            'marker_size': 0.05,
            'dictionary_name': 'DICT_5X5_100',
            'image_topic': 'camera/image_raw',
            'camera_info_topic': 'camera/camera_info',
            'cube_size': 0.10,
            'cube_detections_topic': 'aruco/cubes',
            'debug_image': False,
            'face_vertical_threshold': 0.65,
            'face_switch_hysteresis': 0.15,
            'position_filter_enabled': True,
            'position_filter_alpha': 0.5,
            'position_outlier_threshold': 0.0,
            'position_filter_reset_timeout': 0.5,
            'activation_topic': '/parcel_acquisition/vision_active',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        marker_size = float(self.get_parameter('marker_size').value)
        dictionary_name = self.get_parameter('dictionary_name').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        cube_topic = self.get_parameter('cube_detections_topic').value
        self.debug_image = bool(self.get_parameter('debug_image').value)

        self.pose_solver = MarkerPoseSolver(marker_size)
        self.cube_estimator = CubeEstimator(
            cube_size=float(self.get_parameter('cube_size').value),
            vertical_threshold=float(
                self.get_parameter('face_vertical_threshold').value
            ),
            switch_hysteresis=float(
                self.get_parameter('face_switch_hysteresis').value
            ),
            filter_enabled=bool(
                self.get_parameter('position_filter_enabled').value
            ),
            filter_alpha=float(
                self.get_parameter('position_filter_alpha').value
            ),
            outlier_threshold=float(
                self.get_parameter('position_outlier_threshold').value
            ),
            filter_reset_timeout=float(
                self.get_parameter('position_filter_reset_timeout').value
            ),
        )
        self.marker_axis_length = marker_size * 0.5
        self.cube_axis_length = self.cube_estimator.cube_size * 0.5
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_matrix = None
        self.distortion = None
        self.camera_info_warning_emitted = False
        self.active = False
        self._create_detector(dictionary_name)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, sensor_qos
        )
        self.create_subscription(
            Image, image_topic, self._image_callback, sensor_qos
        )
        activation_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, self.get_parameter('activation_topic').value,
            self._activation_callback, activation_qos,
        )
        self.detection_pub = self.create_publisher(
            Detection3DArray, 'aruco/detections', 10
        )
        self.cube_detection_pub = self.create_publisher(
            CubeDetectionArray, cube_topic, 10
        )
        self.debug_image_pub = None
        self.cube_debug_image_pub = None
        if self.debug_image:
            self.debug_image_pub = self.create_publisher(
                Image, 'aruco/image_detected', sensor_qos
            )
            self.cube_debug_image_pub = self.create_publisher(
                Image, 'aruco/image_cube', sensor_qos
            )
        self.get_logger().info(
            f'ArUco detector idle on {image_topic} using {dictionary_name}'
        )

    def _activation_callback(self, message):
        self.active = bool(message.data)
        self.get_logger().info(
            'ArUco detector active' if self.active else 'ArUco detector idle'
        )

    def _create_detector(self, dictionary_name):
        dictionary_type = getattr(
            cv2.aruco, dictionary_name, cv2.aruco.DICT_5X5_100
        )
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            dictionary = cv2.aruco.getPredefinedDictionary(dictionary_type)
        else:
            dictionary = cv2.aruco.Dictionary_get(dictionary_type)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector_parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                dictionary, self.detector_parameters
            )
        else:
            self.detector_parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
        self.dictionary = dictionary

    def _camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self.distortion = np.array(msg.d, dtype=np.float32)
            self.get_logger().info('Received camera intrinsics.')

    def _image_callback(self, msg):
        if not self.active:
            return
        if self.camera_matrix is None or self.distortion is None:
            if not self.camera_info_warning_emitted:
                self.get_logger().warning('Waiting for CameraInfo...')
                self.camera_info_warning_emitted = True
            return
        marker_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        cube_image = marker_image.copy() if self.debug_image else None
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(marker_image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                marker_image, self.dictionary,
                parameters=self.detector_parameters,
            )

        detections = Detection3DArray()
        detections.header = msg.header
        face_candidates = {}
        if ids is not None:
            for index, marker_id in enumerate(ids.flatten()):
                pose = self.pose_solver.solve(
                    corners[index][0], self.camera_matrix, self.distortion
                )
                if pose is None:
                    continue
                marker_id = int(marker_id)
                detections.detections.append(
                    self._marker_detection(msg.header, marker_id, pose)
                )
                face_candidates.setdefault(marker_id, []).append(
                    self.cube_estimator.face_candidate(pose)
                )
                self._broadcast_marker(msg.header, marker_id, index, pose)
                if self.debug_image:
                    DebugRenderer.draw_marker(
                        marker_image, self.camera_matrix, self.distortion,
                        pose, self.marker_axis_length,
                    )
            if self.debug_image:
                cv2.aruco.drawDetectedMarkers(marker_image, corners)

        self.detection_pub.publish(detections)
        self._publish_cubes(msg.header, face_candidates, cube_image)
        if self.debug_image:
            self._publish_debug_image(
                self.debug_image_pub, marker_image, msg.header
            )
            self._publish_debug_image(
                self.cube_debug_image_pub, cube_image, msg.header
            )

    @staticmethod
    def _marker_detection(header, marker_id, pose):
        quaternion = R.from_matrix(pose.rotation).as_quat()
        tx, ty, tz = pose.tvec.ravel()
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = str(marker_id)
        result.hypothesis.score = 1.0
        result.pose.pose.position.x = float(tx)
        result.pose.pose.position.y = float(ty)
        result.pose.pose.position.z = float(tz)
        result.pose.pose.orientation.x = float(quaternion[0])
        result.pose.pose.orientation.y = float(quaternion[1])
        result.pose.pose.orientation.z = float(quaternion[2])
        result.pose.pose.orientation.w = float(quaternion[3])
        detection = Detection3D()
        detection.header = header
        detection.results.append(result)
        return detection

    def _broadcast_marker(self, header, marker_id, index, pose):
        quaternion = R.from_matrix(pose.rotation).as_quat()
        tx, ty, tz = pose.tvec.ravel()
        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = f'aruco_marker_{marker_id}_face_{index}'
        transform.transform.translation.x = float(tx)
        transform.transform.translation.y = float(ty)
        transform.transform.translation.z = float(tz)
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(transform)

    def _publish_cubes(self, header, candidates, debug_image):
        message = CubeDetectionArray()
        message.header = header
        for marker_id, faces in candidates.items():
            estimate = self.cube_estimator.estimate(marker_id, faces)
            quaternion = R.from_matrix(estimate.rotation).as_quat()
            cube = CubeDetection()
            cube.id = marker_id
            cube.pose.position.x = float(estimate.center[0])
            cube.pose.position.y = float(estimate.center[1])
            cube.pose.position.z = float(estimate.center[2])
            cube.pose.orientation.x = float(quaternion[0])
            cube.pose.orientation.y = float(quaternion[1])
            cube.pose.orientation.z = float(quaternion[2])
            cube.pose.orientation.w = float(quaternion[3])
            cube.face_normal.x = float(estimate.normal[0])
            cube.face_normal.y = float(estimate.normal[1])
            cube.face_normal.z = float(estimate.normal[2])
            cube.face_count = estimate.face_count
            message.detections.append(cube)
            transform = TransformStamped()
            transform.header = header
            transform.child_frame_id = f'cube_{marker_id}'
            transform.transform.translation.x = cube.pose.position.x
            transform.transform.translation.y = cube.pose.position.y
            transform.transform.translation.z = cube.pose.position.z
            transform.transform.rotation = cube.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
            if debug_image is not None:
                DebugRenderer.draw_cube(
                    debug_image, self.camera_matrix, self.distortion,
                    estimate, self.cube_axis_length,
                )
        self.cube_detection_pub.publish(message)

    def _publish_debug_image(self, publisher, image, header):
        message = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
        message.header = header
        publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
