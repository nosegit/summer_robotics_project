#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelPositionTest(Node):
    def __init__(self):
        super().__init__('wheel_position_test')
        self.declare_parameter('joint_name', 'front_left_wheel_joint')
        self.declare_parameter('target_angle', math.pi)
        self.declare_parameter('linear_speed', 0.03)
        self.declare_parameter('timeout', 15.0)

        self.joint_name = self.get_parameter('joint_name').value
        self.target_angle = abs(self.get_parameter('target_angle').value)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.timeout = self.get_parameter('timeout').value

        self.initial_position = None
        self.finished = False
        self.stop_cycles = 0
        self.start_time = self.get_clock().now()

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_timer(0.05, self.control_loop)

    def joint_callback(self, message):
        if self.joint_name not in message.name:
            return

        index = message.name.index(self.joint_name)
        if index >= len(message.position):
            return

        position = message.position[index]
        if self.initial_position is None:
            self.initial_position = position
            self.get_logger().info(
                f'Starting {self.joint_name} at {position:.3f} rad')
            return

        traveled = abs(position - self.initial_position)
        if traveled >= self.target_angle and not self.finished:
            self.finished = True
            self.get_logger().info(
                f'Target reached: {traveled:.3f} rad; stopping')

    def control_loop(self):
        command = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed >= self.timeout and not self.finished:
            self.finished = True
            self.get_logger().error('Timed out before reaching the target angle')

        if self.initial_position is not None and not self.finished:
            command.linear.x = self.linear_speed

        self.publisher.publish(command)

        if self.finished:
            self.stop_cycles += 1
            if self.stop_cycles >= 5:
                rclpy.shutdown()


def main():
    rclpy.init()
    node = WheelPositionTest()
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
