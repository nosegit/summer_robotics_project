"""Parcel acquisition action clients."""

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
