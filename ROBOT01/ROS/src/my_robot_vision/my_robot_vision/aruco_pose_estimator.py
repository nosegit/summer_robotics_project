#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

# --- CHANGED: Imported vision_msgs instead of geometry_msgs PoseArray ---
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose


class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')

        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('dictionary_name', 'DICT_5X5_100')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')

        self.marker_size = self.get_parameter('marker_size').value
        self.dict_name = self.get_parameter('dictionary_name').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.aruco_dict_type = getattr(cv2.aruco, self.dict_name, cv2.aruco.DICT_5X5_100)
        
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            self.dictionary = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        else:
            self.dictionary = cv2.aruco.Dictionary_get(self.aruco_dict_type)

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
        else:
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self.detector = None

        self.camera_matrix = None
        self.dist_coeffs = None

        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, 10
        )

        # --- CHANGED: Publisher now uses Detection3DArray ---
        self.detection_pub = self.create_publisher(Detection3DArray, 'aruco/detections', 10)
        self.debug_image_pub = self.create_publisher(Image, 'aruco/image_detected', 10)

        half_size = self.marker_size / 2.0
        self.marker_3d_edges = np.array([
            [-half_size,  half_size, 0.0],
            [ half_size,  half_size, 0.0],
            [ half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0]
        ], dtype=np.float32)

        self.get_logger().info(
            f'ArUco Node initialized. Listening on {image_topic} with dictionary {self.dict_name}'
        )

    def _camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            self.get_logger().info('Received camera intrinsics from CameraInfo topic.')

    def _image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn_once('Waiting for camera intrinsics (CameraInfo)...')
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.detector:
            corners, ids, _ = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.dictionary, parameters=self.detector_params
            )

        # --- CHANGED: Initialize Detection3DArray ---
        detection_array = Detection3DArray()
        detection_array.header = msg.header

        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]

                success, rvec, tvec = cv2.solvePnP(
                    self.marker_3d_edges,
                    marker_corners,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if not success:
                    continue

                tx, ty, tz = tvec.ravel()
                rot_matrix, _ = cv2.Rodrigues(rvec)
                quat = R.from_matrix(rot_matrix).as_quat()

                # --- CHANGED: Populate Detection3D message with Tag ID and Pose ---
                detection = Detection3D()
                detection.header = msg.header
                
                # Setup hypothesis (holds ID and score)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(marker_id)  # <-- TAG NUMBER SAVED HERE
                hyp.hypothesis.score = 1.0                # 100% confidence
                
                # Setup Pose
                hyp.pose.pose.position.x = float(tx)
                hyp.pose.pose.position.y = float(ty)
                hyp.pose.pose.position.z = float(tz)
                hyp.pose.pose.orientation.x = float(quat[0])
                hyp.pose.pose.orientation.y = float(quat[1])
                hyp.pose.pose.orientation.z = float(quat[2])
                hyp.pose.pose.orientation.w = float(quat[3])
                
                detection.results.append(hyp)
                detection_array.detections.append(detection)

                # Broadcast Dynamic TF
                tf_msg = TransformStamped()
                tf_msg.header.stamp = msg.header.stamp
                tf_msg.header.frame_id = msg.header.frame_id
                tf_msg.child_frame_id = f'aruco_marker_{marker_id}'
                tf_msg.transform.translation.x = float(tx)
                tf_msg.transform.translation.y = float(ty)
                tf_msg.transform.translation.z = float(tz)
                tf_msg.transform.rotation.x = float(quat[0])
                tf_msg.transform.rotation.y = float(quat[1])
                tf_msg.transform.rotation.z = float(quat[2])
                tf_msg.transform.rotation.w = float(quat[3])

                self.tf_broadcaster.sendTransform(tf_msg)

                cv2.aruco.drawDetectedMarkers(cv_image, corners)
                cv2.drawFrameAxes(
                    cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size * 0.5
                )

        # --- CHANGED: Publish the new detection array ---
        self.detection_pub.publish(detection_array)
        
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)


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