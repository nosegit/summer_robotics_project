#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomDistanceTest(Node):
    def __init__(self):
        super().__init__('odom_distance_test')
        self.declare_parameter('target_distance', 0.10)
        self.declare_parameter('linear_speed', 0.03)
        self.declare_parameter('timeout', 15.0)

        self.target_distance = abs(self.get_parameter('target_distance').value)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.timeout = self.get_parameter('timeout').value
        self.initial_x = None
        self.finished = False
        self.stop_cycles = 0
        self.start_time = self.get_clock().now()

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, message):
        current_x = message.pose.pose.position.x
        if self.initial_x is None:
            self.initial_x = current_x
            self.get_logger().info(f'Starting odom X: {self.initial_x:.4f} m')
            return

        direction = 1.0 if self.linear_speed >= 0.0 else -1.0
        traveled = direction * (current_x - self.initial_x)
        if traveled >= self.target_distance and not self.finished:
            self.finished = True
            self.get_logger().info(
                f'Target reached: odom delta X = {traveled:.4f} m; stopping')

    def control_loop(self):
        command = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed >= self.timeout and not self.finished:
            self.finished = True
            self.get_logger().error('Timed out before reaching the odometry target')

        if self.initial_x is not None and not self.finished:
            command.linear.x = self.linear_speed

        self.publisher.publish(command)

        if self.finished:
            self.stop_cycles += 1
            if self.stop_cycles >= 5:
                rclpy.shutdown()


def main():
    rclpy.init()
    node = OdomDistanceTest()
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
