"""Parcel acquisition action clients."""

import rclpy
from action_msgs.msg import GoalStatus
from my_robot_parcel_aquisition.action import RotateBlind, SearchParcel, ServoParcel
from rclpy.action import ActionClient


class ParcelAcquisitionClient:
    def __init__(self, node, actions, pickup_rotation):
        self.node = node
        self.actions = actions
        self.pickup_rotation = pickup_rotation
        self.search_client = ActionClient(node, SearchParcel, 'search_parcel')
        self.servo_client = ActionClient(node, ServoParcel, 'servo_parcel')
        self.rotate_client = ActionClient(node, RotateBlind, 'rotate_blind')

    def acquire(self, marker_id):
        self.search(marker_id)
        self.servo(marker_id)
        self.rotate()
        self.node.get_logger().info('Parcel acquisition positioning complete')

    def search(self, marker_id):
        self.actions.wait_for_server(self.search_client, 'search_parcel')
        result = self.actions.run(
            self.search_client, SearchParcel.Goal(marker_id=marker_id),
            'parcel search', feedback_name='search',
        )
        if not result.success:
            raise RuntimeError(f'parcel search failed: {result.message}')

    def search_until_y(self, marker_id, get_map_y, limit_y):
        """Search until the parcel is found or the robot reaches a map Y limit."""
        self.actions.wait_for_server(self.search_client, 'search_parcel')
        feedback_callback = None
        if self.actions.show_feedback:
            feedback_callback = self.actions._feedback_callback('search')
        goal_future = self.search_client.send_goal_async(
            SearchParcel.Goal(marker_id=marker_id),
            feedback_callback=feedback_callback,
        )
        rclpy.spin_until_future_complete(self.node, goal_future)
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('parcel search goal was rejected')

        result_future = handle.get_result_async()
        crossed_limit = False
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if result_future.done():
                break
            map_y = get_map_y()
            if map_y is not None and map_y >= limit_y:
                crossed_limit = True
                self.node.get_logger().info(
                    f'Parcel search reached map y={map_y:.3f}; '
                    f'stopping at boundary y={limit_y:.3f}'
                )
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, cancel_future)
                break

        rclpy.spin_until_future_complete(self.node, result_future)
        wrapped = result_future.result()
        if crossed_limit:
            return False
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if wrapped is None else wrapped.status
            raise RuntimeError(f'parcel search failed with action status {status}')
        if not wrapped.result.success:
            raise RuntimeError(
                f'parcel search failed: {wrapped.result.message}'
            )
        return True

    def servo(self, marker_id):
        self.actions.wait_for_server(self.servo_client, 'servo_parcel')
        result = self.actions.run(
            self.servo_client, ServoParcel.Goal(marker_id=marker_id),
            'parcel docking', feedback_name='servo',
        )
        if not result.success:
            raise RuntimeError(f'parcel docking failed: {result.message}')

    def rotate(self):
        self.actions.wait_for_server(self.rotate_client, 'rotate_blind')
        result = self.actions.run(
            self.rotate_client, RotateBlind.Goal(angle=self.pickup_rotation),
            'parcel pickup positioning', feedback_name='rotate',
        )
        if not result.success:
            raise RuntimeError(f'parcel pickup failed: {result.message}')
