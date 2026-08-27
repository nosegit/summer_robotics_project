"""Nav2 pose and initial-localization helpers."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class NavigationClient:
    def __init__(self, node, config, actions):
        self.node = node
        self.config = config
        self.actions = actions
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_publisher = node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

    def wait_until_ready(self):
        self.actions.wait_for_server(self.client, 'navigate_to_pose')

    def _pose_stamped(self, values):
        pose = PoseStamped()
        pose.header.frame_id = self.config.map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = values[0]
        pose.pose.position.y = values[1]
        half_yaw = values[2] * 0.5
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def set_initial_pose(self):
        count = self.config.initial_pose_publish_count
        period = self.config.initial_pose_publish_period
        if count < 1 or period < 0.0:
            raise ValueError(
                'Initial-pose publish count must be positive and period non-negative'
            )
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.config.map_frame
        message.pose.pose = self._pose_stamped(self.config.start_pose).pose
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self.node.get_logger().info(
            f'Setting fixed initial pose to {self.config.start_pose} '
            f'in {self.config.map_frame}'
        )
        for _ in range(count):
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.initial_pose_publisher.publish(message)
            self.wait(period, log=False)
        self.wait(max(0.0, self.config.localization_settle_time), log=False)

    def navigate_to(self, pose, name):
        self.node.get_logger().info(f'Navigating to {name}: {pose}')
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(pose)
        self.actions.run(self.client, goal, f'navigation to {name}')
        self.node.get_logger().info(f'Arrived at {name}')

    def wait(self, seconds, log=True):
        if log:
            self.node.get_logger().info(f'Waiting for {seconds} seconds')
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.1, deadline - time.monotonic()),
            )
