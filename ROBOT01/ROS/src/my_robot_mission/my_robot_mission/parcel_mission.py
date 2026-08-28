"""Top-level orchestration for YAML-defined parcel missions."""

import argparse
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from .utils.action_utils import ActionRunner
from .utils.door_utils import DoorClient
from .utils.drone_utils import DroneParcelSelector
from .utils.nav_utils import NavigationClient
from .utils.parcel_utils import ParcelAcquisitionClient
from .utils.service_utils import GripperClient, MissionLedClient
from .utils.yaml_utils import load_mission_file, parse_mission_config


class ParcelMission(Node):
    """Execute validated mission commands through subsystem clients."""

    def __init__(self, config, show_feedback=False):
        super().__init__('parcel_mission')
        self.config = (
            config if hasattr(config, 'commands') else parse_mission_config(config)
        )
        self.actions = ActionRunner(
            self, self.config.server_timeout, show_feedback
        )
        self.navigation = NavigationClient(self, self.config, self.actions)
        self.parcel = ParcelAcquisitionClient(
            self, self.actions, self.config.pickup_rotation
        )
        self.gripper = GripperClient(self, self.config.server_timeout)
        self.led = MissionLedClient(self, self.config.server_timeout)
        self.doors = DoorClient(self, self.config.doors)
        self._map_y = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._map_pose_callback,
            10,
        )
        self.drone = None
        if self.config.drone_parcel is not None:
            self.drone = DroneParcelSelector(
                self,
                self.config.drone_parcel,
            )

    def _map_pose_callback(self, message):
        self._map_y = float(message.pose.pose.position.y)

    def _search_parcel_two_areas(self, marker_id):
        """Search right area up to y=-4, then restart from the left area."""
        right_pose = 'parcel_right'
        left_pose = 'parcel_left'
        boundary_y = -4.0
        missing = [
            name for name in (right_pose, left_pose)
            if name not in self.config.poses
        ]
        if missing:
            raise RuntimeError(
                f'two-area parcel search requires poses: {missing}'
            )

        self.navigation.navigate_to(
            self.config.poses[right_pose], 'parcel right search start'
        )
        if self._map_y is None:
            raise RuntimeError(
                'two-area parcel search requires localization on /amcl_pose'
            )
        found = self.parcel.search_until_y(
            marker_id, lambda: self._map_y, boundary_y
        )
        if found:
            return

        self.get_logger().info(
            'Parcel not found in right area; restarting search from parcel_left'
        )
        self.navigation.navigate_to(
            self.config.poses[left_pose], 'parcel left search start'
        )
        self.parcel.search(marker_id)

    def _navigate_to_drone_parcel(self, marker_id):
        pose_key, median_y = self.drone.select_staging_pose(marker_id)
        self.get_logger().info(
            f'Drone selected {pose_key!r} for marker {marker_id} '
            f'(median map y={median_y:.3f})'
        )
        self.navigation.navigate_to(
            self.config.poses[pose_key],
            f'drone-selected parcel staging pose {pose_key}',
        )

    def _search_fallback(self, marker_id, restart_pose):
        """Retry a failed search only after returning to a safe pose."""
        attempts = self.config.retry_count + 1
        for attempt in range(1, attempts + 1):
            try:
                self.parcel.search(marker_id)
                return
            except RuntimeError as error:
                if attempt == attempts:
                    raise
                self.get_logger().warning(
                    f'Parcel search failed on attempt {attempt}/{attempts} '
                    f'({error}); returning to {restart_pose!r} before retry'
                )
                self.navigation.navigate_to(
                    self.config.poses[restart_pose],
                    f'parcel search restart pose {restart_pose}',
                )

    def _execute(self, command):
        name = command.name
        argument = command.argument
        if name == 'set_initial_pose':
            self.navigation.set_initial_pose()
        elif name == 'navigate_to_pose':
            self.navigation.navigate_to(
                self.config.poses[argument], f'pose {argument}'
            )
        elif name == 'navigate_to_drone_parcel':
            self._navigate_to_drone_parcel(argument)
        elif name == 'acquire_parcel':
            self.parcel.acquire(argument)
        elif name == 'search_parcel':
            self.parcel.search(argument)
        elif name == 'search_fallback':
            self._search_fallback(*argument)
        elif name == 'search_parcel_two_areas':
            self._search_parcel_two_areas(argument)
        elif name == 'servo_parcel':
            self.parcel.servo(argument)
        elif name == 'rotate_blind':
            self.parcel.rotate()
        elif name in {'idle', 'open_gripper', 'pickup'}:
            self.gripper.run(name)
        elif name == 'open_door':
            self.doors.send(argument, 'open')
        elif name == 'close_door':
            self.doors.send(argument, 'close')
        elif name == 'wait':
            self.navigation.wait(argument)
        elif name == 'led_blink':
            self.led.set('blinking')
        elif name == 'led_red':
            self.led.set('solid red')
        elif name == 'led_green':
            self.led.set('solid green')
        elif name == 'led_off':
            self.led.set('off')

    def _execute_with_retry(self, command):
        search_commands = {
            'search_parcel', 'search_fallback',
            'search_parcel_two_areas', 'acquire_parcel',
        }
        attempts = (
            1 if command.name in search_commands
            else self.config.retry_count + 1
        )
        for attempt in range(1, attempts + 1):
            try:
                self._execute(command)
                return
            except Exception as error:
                if attempt == attempts:
                    raise
                self.get_logger().warning(
                    f'Command {command.text} failed on attempt {attempt}/'
                    f'{attempts}: {error}; retrying'
                )

    def run(self):
        if not self.config.configured:
            raise RuntimeError(
                'mission_configured is false; set safe measured poses in '
                'mission.yaml before enabling the mission'
            )
        self.get_logger().info(
            'Mission: ' + ' -> '.join(
                command.text for command in self.config.commands
            )
        )
        self.led.set('blinking')
        navigation_commands = {
            'navigate_to_pose', 'navigate_to_drone_parcel',
            'search_fallback', 'search_parcel_two_areas',
        }
        if any(
            command.name in navigation_commands
            for command in self.config.commands
        ):
            self.navigation.wait_until_ready()

        total = len(self.config.commands)
        for index, command in enumerate(self.config.commands, start=1):
            self.get_logger().info(
                f'Running command {index}/{total}: {command.text}'
            )
            self._execute_with_retry(command)

        self.led.set('solid red')
        self.get_logger().info('Mission complete')


def main(args=None):
    default_config = os.path.join(
        get_package_share_directory('my_robot_mission'),
        'config',
        'mission.yaml',
    )
    parser = argparse.ArgumentParser(description='Run a parcel mission')
    parser.add_argument('--mission-file', default=default_config)
    parser.add_argument(
        '--feedback', action='store_true',
        help='Print feedback from parcel search, servo, and rotate actions',
    )
    parsed, ros_args = parser.parse_known_args(
        sys.argv[1:] if args is None else args
    )
    config = load_mission_file(parsed.mission_file)
    rclpy.init(args=ros_args)
    node = None
    exit_code = 0
    try:
        node = ParcelMission(config, show_feedback=parsed.feedback)
        node.run()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as error:
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
