import argparse
import math
import os
import re
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from my_robot_parcel_aquisition.action import RotateBlind, SearchParcel, ServoParcel
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


class ParcelMission(Node):
    """Execute the command list from a mission YAML file."""

    COMMAND_PATTERN = re.compile(
        r'^(set_initial_pose|navigate_to_pose|acquire_parcel|search_parcel|'
        r'servo_parcel|rotate_blind|idle|open_gripper|pickup)'
        r'(?:\(\s*(-?\d+)\s*\))?$'
    )

    def __init__(self, config):
        super().__init__('parcel_mission')
        self._configured = bool(config.get('mission_configured', False))
        self._start_pose = self._validate_pose(
            config.get('start_pose'), 'start_pose'
        )
        raw_poses = config.get('poses', {})
        if not isinstance(raw_poses, dict):
            raise ValueError('poses must be a YAML mapping of ID to [x, y, yaw]')
        self._poses = {
            int(pose_id): self._validate_pose(values, f'poses[{pose_id}]')
            for pose_id, values in raw_poses.items()
        }
        self._commands = self._parse_commands(config.get('commands'))
        self._pickup_rotation = float(config.get('pickup_rotation', -1.5708))
        self._initial_pose_publish_count = int(
            config.get('initial_pose_publish_count', 5)
        )
        self._initial_pose_publish_period = float(
            config.get('initial_pose_publish_period', 0.5)
        )
        self._localization_settle_time = float(
            config.get('localization_settle_time', 2.0)
        )
        self._server_timeout = float(config.get('server_timeout', 60.0))
        self._map_frame = str(config.get('map_frame', 'map'))
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self._navigate = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._search = ActionClient(self, SearchParcel, 'search_parcel')
        self._servo = ActionClient(self, ServoParcel, 'servo_parcel')
        self._rotate = ActionClient(self, RotateBlind, 'rotate_blind')
        self._gripper_idle = self.create_client(
            Trigger, '/gripper_controller/idle'
        )
        self._gripper_pickup = self.create_client(
            Trigger, '/gripper_controller/pickup'
        )
        self._gripper_open = self.create_client(
            Trigger, '/gripper_controller/open_gripper'
        )
        self._gripper_status = self.create_client(
            Trigger, '/gripper_controller/status'
        )

    @staticmethod
    def _validate_pose(value, name):
        if not isinstance(value, (list, tuple)):
            raise ValueError(f'{name} must be [x, y, yaw]')
        values = list(value)
        if len(values) != 3 or not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f'{name} must contain three finite values: [x, y, yaw]')
        return tuple(float(value) for value in values)

    def _parse_commands(self, commands):
        if not isinstance(commands, list) or not commands:
            raise ValueError('commands must be a non-empty YAML list')
        parsed = []
        for command in commands:
            text = str(command).strip()
            match = self.COMMAND_PATTERN.fullmatch(text)
            if match is None:
                raise ValueError(f'Invalid mission command: {text!r}')
            name, argument = match.groups()
            no_argument_commands = {
                'set_initial_pose', 'rotate_blind', 'idle',
                'open_gripper', 'pickup',
            }
            marker_or_pose_commands = {
                'navigate_to_pose', 'acquire_parcel',
                'search_parcel', 'servo_parcel',
            }
            if name in no_argument_commands and argument is not None:
                raise ValueError(f'{name} does not accept an argument')
            if name in marker_or_pose_commands and argument is None:
                raise ValueError(f'{name} requires an integer argument')
            parsed.append((text, name, None if argument is None else int(argument)))

        missing_poses = sorted({
            argument for _, name, argument in parsed
            if name == 'navigate_to_pose' and argument not in self._poses
        })
        if missing_poses:
            raise ValueError(f'Commands reference undefined pose IDs: {missing_poses}')
        return parsed

    @staticmethod
    def _orientation(yaw):
        half_yaw = yaw * 0.5
        return math.sin(half_yaw), math.cos(half_yaw)

    def _pose_stamped(self, values):
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = values[0]
        pose.pose.position.y = values[1]
        pose.pose.orientation.z, pose.pose.orientation.w = self._orientation(values[2])
        return pose

    def _wait_for_server(self, client, name):
        self.get_logger().info(f'Waiting for {name} action server...')
        if not client.wait_for_server(timeout_sec=self._server_timeout):
            raise RuntimeError(f'{name} action server is unavailable')

    def _set_initial_pose(self):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self._map_frame
        message.pose.pose = self._pose_stamped(self._start_pose).pose
        # AMCL convention: uncertainty in x, y, and yaw.
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        count = self._initial_pose_publish_count
        period = self._initial_pose_publish_period
        if count < 1 or period < 0.0:
            raise ValueError('Initial-pose publish count must be positive and period non-negative')
        self.get_logger().info(
            f'Setting fixed initial pose to {self._start_pose} in {self._map_frame}'
        )
        for _ in range(count):
            message.header.stamp = self.get_clock().now().to_msg()
            self._initial_pose_publisher.publish(message)
            end_time = time.monotonic() + period
            while rclpy.ok() and time.monotonic() < end_time:
                rclpy.spin_once(
                    self, timeout_sec=min(0.1, end_time - time.monotonic())
                )
        settle_time = self._localization_settle_time
        end_time = time.monotonic() + max(0.0, settle_time)
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=min(0.1, end_time - time.monotonic()))

    def _run_action(self, client, goal, name):
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'{name} goal was rejected')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if wrapped_result is None else wrapped_result.status
            raise RuntimeError(f'{name} failed with action status {status}')
        return wrapped_result.result

    def _run_gripper_sequence(self, client, sequence_name):
        self.get_logger().info(f'Starting gripper {sequence_name} sequence')
        if not client.wait_for_service(timeout_sec=self._server_timeout):
            raise RuntimeError(
                f'gripper {sequence_name} service is unavailable'
            )
        if not self._gripper_status.wait_for_service(
            timeout_sec=self._server_timeout
        ):
            raise RuntimeError('gripper status service is unavailable')

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            raise RuntimeError(
                f'gripper {sequence_name} was rejected: {message}'
            )

        deadline = time.monotonic() + self._server_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            status_future = self._gripper_status.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, status_future)
            status = status_future.result()
            if status is not None and status.success:
                self.get_logger().info(
                    f'Gripper {sequence_name} sequence completed'
                )
                return
            time.sleep(0.05)
        raise RuntimeError(f'gripper {sequence_name} sequence timed out')

    def _navigate_to(self, pose, name):
        self.get_logger().info(f'Navigating to {name}: {pose}')
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(pose)
        self._run_action(self._navigate, goal, f'navigation to {name}')
        self.get_logger().info(f'Arrived at {name}')

    def _acquire_parcel(self, marker_id):
        self._search_for_parcel(marker_id)
        self._servo_to_parcel(marker_id)
        self._rotate_for_pickup()
        self.get_logger().info('Parcel acquisition positioning complete')

    def _search_for_parcel(self, marker_id):
        self._wait_for_server(self._search, 'search_parcel')
        result = self._run_action(
            self._search, SearchParcel.Goal(marker_id=marker_id), 'parcel search'
        )
        if not result.success:
            raise RuntimeError(f'parcel search failed: {result.message}')

    def _servo_to_parcel(self, marker_id):
        self._wait_for_server(self._servo, 'servo_parcel')
        result = self._run_action(
            self._servo, ServoParcel.Goal(marker_id=marker_id), 'parcel docking'
        )
        if not result.success:
            raise RuntimeError(f'parcel docking failed: {result.message}')

    def _rotate_for_pickup(self):
        self._wait_for_server(self._rotate, 'rotate_blind')
        result = self._run_action(
            self._rotate,
            RotateBlind.Goal(angle=self._pickup_rotation),
            'parcel pickup positioning',
        )
        if not result.success:
            raise RuntimeError(f'parcel pickup failed: {result.message}')

    def run(self):
        if not self._configured:
            raise RuntimeError(
                'mission_configured is false; set safe measured poses in '
                'mission.yaml before enabling the mission'
            )
        command_text = [command[0] for command in self._commands]
        self.get_logger().info(f'Mission: {" -> ".join(command_text)}')
        if any(name == 'navigate_to_pose' for _, name, _ in self._commands):
            self._wait_for_server(self._navigate, 'navigate_to_pose')

        for index, (text, name, argument) in enumerate(self._commands, start=1):
            self.get_logger().info(
                f'Running command {index}/{len(self._commands)}: {text}'
            )
            if name == 'set_initial_pose':
                self._set_initial_pose()
            elif name == 'navigate_to_pose':
                self._navigate_to(self._poses[argument], f'pose {argument}')
            elif name == 'acquire_parcel':
                self._acquire_parcel(argument)
            elif name == 'search_parcel':
                self._search_for_parcel(argument)
            elif name == 'servo_parcel':
                self._servo_to_parcel(argument)
            elif name == 'rotate_blind':
                self._rotate_for_pickup()
            elif name == 'idle':
                self._run_gripper_sequence(self._gripper_idle, 'idle')
            elif name == 'open_gripper':
                self._run_gripper_sequence(
                    self._gripper_open, 'open_gripper'
                )
            elif name == 'pickup':
                self._run_gripper_sequence(self._gripper_pickup, 'pickup')

        self.get_logger().info('Mission complete')


def main(args=None):
    default_config = os.path.join(
        get_package_share_directory('my_robot_mission'),
        'config',
        'mission.yaml',
    )
    parser = argparse.ArgumentParser(description='Run a parcel mission')
    parser.add_argument('--mission-file', default=default_config)
    parsed, ros_args = parser.parse_known_args(
        sys.argv[1:] if args is None else args
    )
    with open(parsed.mission_file, encoding='utf-8') as config_stream:
        config = yaml.safe_load(config_stream)
    if not isinstance(config, dict):
        raise ValueError('Mission file must contain a YAML mapping')
    rclpy.init(args=ros_args)
    node = None
    exit_code = 0
    try:
        node = ParcelMission(config)
        node.run()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as error:  # Return failure and make the cause visible.
        exit_code = 1
        if node is not None:
            node.get_logger().error(f'Mission aborted: {error}')
        else:
            print(f'Mission aborted: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
