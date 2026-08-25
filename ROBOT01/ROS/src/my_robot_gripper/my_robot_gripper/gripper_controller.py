#!/usr/bin/env python3

from collections import deque
from functools import partial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class SmoothServo:
    def __init__(self, publisher, initial_angle, speed, tolerance):
        self.publisher = publisher
        self.angle = initial_angle
        self.speed = speed
        self.tolerance = tolerance
        self.targets = deque()

    def command(self, targets):
        self.targets = deque(float(target) for target in targets)

    def update_from_state(self, angle):
        if not self.targets:
            self.angle = angle

    def update(self, period):
        if not self.targets:
            return

        target = self.targets[0]
        error = target - self.angle
        max_step = self.speed * period
        if abs(error) <= max(self.tolerance, max_step):
            self.angle = target
            self.targets.popleft()
        else:
            self.angle += max(-max_step, min(max_step, error))

        message = Float64MultiArray()
        message.data = [self.angle]
        self.publisher.publish(message)


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        defaults = {
            'servo_L_topic': '/servo_L/commands',
            'servo_R_topic': '/servo_R/commands',
            'servo_L_joint': 'servo_L_joint',
            'servo_R_joint': 'servo_R_joint',
            'open_angle': 0.0,
            'close_angle': 110.0,
            'down_angle': 80.0,
            'down_fit_angle': 70.0,
            'up_peak_angle': 180.0,
            'up_settle_angle': 140.0,
            'initial_servo_L_angle': 110.0,
            'initial_servo_R_angle': 80.0,
            'gripper_speed': 90.0,
            'lift_speed': 60.0,
            'update_rate': 50.0,
            'angle_tolerance': 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        update_rate = float(self.get_parameter('update_rate').value)
        if update_rate <= 0.0:
            raise ValueError('update_rate must be greater than zero')
        self.period = 1.0 / update_rate

        left_publisher = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('servo_L_topic').value,
            10,
        )
        right_publisher = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('servo_R_topic').value,
            10,
        )
        tolerance = float(self.get_parameter('angle_tolerance').value)
        self.left_servo = SmoothServo(
            left_publisher,
            float(self.get_parameter('initial_servo_L_angle').value),
            float(self.get_parameter('gripper_speed').value),
            tolerance,
        )
        self.right_servo = SmoothServo(
            right_publisher,
            float(self.get_parameter('initial_servo_R_angle').value),
            float(self.get_parameter('lift_speed').value),
            tolerance,
        )

        self.left_joint = self.get_parameter('servo_L_joint').value
        self.right_joint = self.get_parameter('servo_R_joint').value
        self.create_subscription(JointState, '/joint_states', self._joint_state, 10)

        commands = {
            'open': (self.left_servo, ['open_angle']),
            'close': (self.left_servo, ['close_angle']),
            'lift_down': (self.right_servo, ['down_angle']),
            'lift_down_fit': (self.right_servo, ['down_fit_angle']),
            'lift_up': (
                self.right_servo,
                ['up_peak_angle', 'up_settle_angle'],
            ),
        }
        for service_name, (servo, parameter_names) in commands.items():
            self.create_service(
                Trigger,
                f'~/{service_name}',
                partial(
                    self._command,
                    servo=servo,
                    parameter_names=parameter_names,
                    command_name=service_name,
                ),
            )

        self.create_timer(self.period, self._update)

    def _joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if self.left_joint in positions:
            self.left_servo.update_from_state(positions[self.left_joint])
        if self.right_joint in positions:
            self.right_servo.update_from_state(positions[self.right_joint])

    def _command(
        self, request, response, servo, parameter_names, command_name
    ):
        del request
        targets = [
            float(self.get_parameter(parameter_name).value)
            for parameter_name in parameter_names
        ]
        servo.command(targets)
        response.success = True
        response.message = (
            f'{command_name} motion accepted: '
            + ' -> '.join(f'{target:.1f} deg' for target in targets)
        )
        return response

    def _update(self):
        self.left_servo.update(self.period)
        self.right_servo.update(self.period)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
