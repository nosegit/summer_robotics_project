"""Mission clients for gripper and LED Trigger services."""

import time

import rclpy
from std_srvs.srv import Trigger


class GripperClient:
    def __init__(self, node, timeout):
        self.node = node
        self.timeout = timeout
        self.clients = {
            name: node.create_client(Trigger, f'/gripper_controller/{name}')
            for name in ('idle', 'open_gripper', 'pickup')
        }
        self.status = node.create_client(Trigger, '/gripper_controller/status')

    def run(self, sequence):
        client = self.clients[sequence]
        self.node.get_logger().info(f'Starting gripper {sequence} sequence')
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f'gripper {sequence} service is unavailable')
        if not self.status.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError('gripper status service is unavailable')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            raise RuntimeError(f'gripper {sequence} was rejected: {message}')
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            future = self.status.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future)
            status = future.result()
            if status is not None and status.success:
                self.node.get_logger().info(
                    f'Gripper {sequence} sequence completed'
                )
                return
            time.sleep(0.05)
        raise RuntimeError(f'gripper {sequence} sequence timed out')


class MissionLedClient:
    SERVICES = {
        'blinking': 'start_blinking', 'solid red': 'solid_red',
        'solid green': 'solid_green', 'off': 'off',
    }

    def __init__(self, node, timeout):
        self.node = node
        self.timeout = timeout
        self.clients = {
            state: node.create_client(
                Trigger, f'/mission_led_controller/{service}'
            )
            for state, service in self.SERVICES.items()
        }

    def set(self, state):
        client = self.clients[state]
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f'mission LED {state} service is unavailable')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            raise RuntimeError(f'failed to set mission LED {state}: {message}')
