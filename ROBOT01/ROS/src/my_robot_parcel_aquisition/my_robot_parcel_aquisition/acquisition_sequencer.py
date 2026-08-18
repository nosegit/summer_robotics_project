import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_parcel_aquisition.action import RotateBlind, SearchParcel, ServoParcel


class AcquisitionSequencer(Node):
    def __init__(self, marker_id, angle, show_feedback=False):
        super().__init__('parcel_acquisition_sequencer')
        self.show_feedback = show_feedback
        self.stages = (
            (ActionClient(self, SearchParcel, 'search_parcel'), SearchParcel.Goal(marker_id=marker_id), 'search'),
            (ActionClient(self, ServoParcel, 'servo_parcel'), ServoParcel.Goal(marker_id=marker_id), 'servo'),
            (ActionClient(self, RotateBlind, 'rotate_blind'), RotateBlind.Goal(angle=angle), 'rotate'),
        )

    def run(self):
        for client, goal, name in self.stages:
            if not client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f'{name} action server is unavailable')
            feedback_callback = None
            if self.show_feedback:
                feedback_callback = self._feedback_callback(name)
            future = client.send_goal_async(
                goal, feedback_callback=feedback_callback
            )
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

    def _feedback_callback(self, stage):
        def callback(message):
            feedback = message.feedback
            values = ', '.join(
                f'{field}={getattr(feedback, field)}'
                for field in feedback.get_fields_and_field_types()
            )
            self.get_logger().info(f'{stage} feedback: {values}')
        return callback


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--marker-id', type=int, required=True)
    parser.add_argument('--angle', type=float, default=-1.5708)
    parser.add_argument(
        '--feedback', action='store_true',
        help='Print feedback from every acquisition action'
    )
    parsed, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    node = AcquisitionSequencer(
        parsed.marker_id, parsed.angle, show_feedback=parsed.feedback
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
