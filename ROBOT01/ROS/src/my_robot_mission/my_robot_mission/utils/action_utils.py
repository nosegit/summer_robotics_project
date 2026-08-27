"""Shared ROS action execution helpers."""

import rclpy
from action_msgs.msg import GoalStatus


class ActionRunner:
    def __init__(self, node, timeout, show_feedback=False):
        self.node = node
        self.timeout = timeout
        self.show_feedback = show_feedback

    def wait_for_server(self, client, name):
        self.node.get_logger().info(f'Waiting for {name} action server...')
        if not client.wait_for_server(timeout_sec=self.timeout):
            raise RuntimeError(f'{name} action server is unavailable')

    def run(self, client, goal, name, feedback_name=None):
        callback = None
        if self.show_feedback and feedback_name is not None:
            callback = self._feedback_callback(feedback_name)
        future = client.send_goal_async(goal, feedback_callback=callback)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'{name} goal was rejected')
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, future)
        wrapped = future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if wrapped is None else wrapped.status
            raise RuntimeError(f'{name} failed with action status {status}')
        return wrapped.result

    def _feedback_callback(self, stage):
        def callback(message):
            feedback = message.feedback
            values = ', '.join(
                f'{field}={getattr(feedback, field)}'
                for field in feedback.get_fields_and_field_types()
            )
            self.node.get_logger().info(f'{stage} feedback: {values}')
        return callback
