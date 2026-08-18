import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from my_robot_vision.msg import CubeDetectionArray


class AcquisitionManager(Node):
    def __init__(self):
        super().__init__('parcel_acquisition_manager')
        defaults = {
            'cmd_vel_topic': '/cmd_vel', 'cube_detections_topic': '/aruco/cubes',
            'base_frame': 'base_link', 'detection_timeout': 0.5,
            'search_speed': 0.08, 'search_timeout': 30.0,
            'search_center_kp': 0.5, 'search_center_tolerance': 0.05,
            'search_center_settle_cycles': 3,
            'search_reacquire_timeout': 2.0,
            'servo_rate': 20.0, 'servo_timeout': 20.0,
            'target_x': 0.0, 'target_y': 0.20, 'target_yaw': -1.57079632679,
            'kp_x': 0.8, 'kp_y': 0.8, 'kp_yaw': 1.0,
            'position_tolerance': 0.01, 'yaw_tolerance': 0.03,
            'settle_cycles': 5, 'max_linear_velocity': 0.15,
            'max_angular_velocity': 0.5, 'rotate_speed': 0.3,
            'odometry_topic': '/odometry/wheel', 'odometry_timeout': 0.5,
            'rotate_kp': 1.0, 'rotate_min_speed': 0.08,
            'rotate_tolerance': 0.02, 'rotate_timeout': 20.0,
            'rotate_rate': 20.0, 'rotate_settle_cycles': 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.velocity_publisher = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )
        self._detections = {}
        self._detections_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._odometry = None
        self._odometry_lock = threading.Lock()
        self.create_subscription(
            CubeDetectionArray, self.get_parameter('cube_detections_topic').value,
            self._detections_callback, 10, callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value,
            self._odometry_callback, 10, callback_group=self.callback_group,
        )

    def _odometry_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self._odometry_lock:
            self._odometry = (yaw, time.monotonic())

    def get_odometry_yaw(self):
        with self._odometry_lock:
            odometry = self._odometry
        if odometry is None:
            return None
        age = time.monotonic() - odometry[1]
        timeout = float(self.get_parameter('odometry_timeout').value)
        return odometry[0] if age <= timeout else None

    def _detections_callback(self, msg):
        received = time.monotonic()
        with self._detections_lock:
            for detection in msg.detections:
                self._detections[detection.id] = (detection, msg.header, received)

    def get_detection(self, marker_id):
        with self._detections_lock:
            item = self._detections.get(marker_id)
        if item is None:
            return None
        age = time.monotonic() - item[2]
        timeout = float(self.get_parameter('detection_timeout').value)
        return item if age <= timeout else None

    def try_claim_motion(self):
        return self._motion_lock.acquire(blocking=False)

    def release_motion(self):
        self._motion_lock.release()

    def publish_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        command = Twist()
        command.linear.x = float(vx)
        command.linear.y = float(vy)
        command.angular.z = float(wz)
        self.velocity_publisher.publish(command)

    def stop(self):
        self.publish_velocity()
