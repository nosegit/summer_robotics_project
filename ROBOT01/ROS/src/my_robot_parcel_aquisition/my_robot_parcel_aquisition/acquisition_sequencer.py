import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_parcel_aquisition.action import RotateBlind, SearchParcel, ServoParcel


class AcquisitionSequencer(Node):
    def __init__(self, marker_id, angle):
        super().__init__('parcel_acquisition_sequencer')
        self.stages = (
            (ActionClient(self, SearchParcel, 'search_parcel'), SearchParcel.Goal(marker_id=marker_id), 'search'),
            (ActionClient(self, ServoParcel, 'servo_parcel'), ServoParcel.Goal(marker_id=marker_id), 'servo'),
            (ActionClient(self, RotateBlind, 'rotate_blind'), RotateBlind.Goal(angle=angle), 'rotate'),
        )

    def run(self):
        for client, goal, name in self.stages:
            if not client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f'{name} action server is unavailable')
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError(f'{name} goal was rejected')
            future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, future)
            result = future.result().result
            if not result.success:
                raise RuntimeError(f'{name} failed: {result.message}')
            self.get_logger().info(f'{name}: {result.message}')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--marker-id', type=int, required=True)
    parser.add_argument('--angle', type=float, default=-1.5708)
    parsed, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    node = AcquisitionSequencer(parsed.marker_id, parsed.angle)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
