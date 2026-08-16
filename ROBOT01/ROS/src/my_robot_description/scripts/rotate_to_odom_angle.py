#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomAngleTest(Node):
    def __init__(self):
        super().__init__('odom_angle_test')
        self.declare_parameter('target_angle', 2.0 * math.pi)
        self.declare_parameter('angular_speed', 0.20)
        self.declare_parameter('timeout', 45.0)

        self.target_angle = abs(self.get_parameter('target_angle').value)
        self.angular_speed = self.get_parameter('angular_speed').value
        self.timeout = self.get_parameter('timeout').value
        self.last_yaw = None
        self.accumulated_yaw = 0.0
        self.finished = False
        self.stop_cycles = 0
        self.start_time = self.get_clock().now()

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.05, self.control_loop)

    @staticmethod
    def yaw_from_odometry(message):
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(sin_yaw, cos_yaw)

    def odom_callback(self, message):
        yaw = self.yaw_from_odometry(message)
        if self.last_yaw is None:
            self.last_yaw = yaw
            self.get_logger().info(f'Starting odom yaw: {yaw:.4f} rad')
            return

        delta = math.atan2(
            math.sin(yaw - self.last_yaw),
            math.cos(yaw - self.last_yaw),
        )
        self.accumulated_yaw += delta
        self.last_yaw = yaw

        direction = 1.0 if self.angular_speed >= 0.0 else -1.0
        traveled = direction * self.accumulated_yaw
        if traveled >= self.target_angle and not self.finished:
            self.finished = True
            self.get_logger().info(
                f'Target reached: accumulated yaw = {traveled:.4f} rad; stopping')

    def control_loop(self):
        command = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed >= self.timeout and not self.finished:
            self.finished = True
            self.get_logger().error('Timed out before reaching the odometry target')

        if self.last_yaw is not None and not self.finished:
            command.angular.z = self.angular_speed

        self.publisher.publish(command)

        if self.finished:
            self.stop_cycles += 1
            if self.stop_cycles >= 5:
                rclpy.shutdown()


def main():
    rclpy.init()
    node = OdomAngleTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publisher.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
