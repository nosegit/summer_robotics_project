import math
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from my_robot_parcel_aquisition.action import RotateBlind
from my_robot_parcel_aquisition.geometry import wrap_angle


class RotateAction:
    def __init__(self, manager):
        self.manager = manager
        self.server = ActionServer(
            manager, RotateBlind, 'rotate_blind', self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=manager.callback_group,
        )

    @staticmethod
    def goal(request):
        return GoalResponse.ACCEPT if math.isfinite(request.angle) else GoalResponse.REJECT

    @staticmethod
    def cancel(_handle):
        return CancelResponse.ACCEPT

    def execute(self, handle):
        result = RotateBlind.Result()
        if not self.manager.try_claim_motion():
            result.message = 'Another acquisition action owns velocity control'
            handle.abort()
            return result
        angle = float(handle.request.angle)
        max_speed = abs(float(self.manager.get_parameter('rotate_speed').value))
        min_speed = abs(float(self.manager.get_parameter('rotate_min_speed').value))
        kp = float(self.manager.get_parameter('rotate_kp').value)
        tolerance = float(self.manager.get_parameter('rotate_tolerance').value)
        timeout = float(self.manager.get_parameter('rotate_timeout').value)
        rate = float(self.manager.get_parameter('rotate_rate').value)
        required_cycles = int(
            self.manager.get_parameter('rotate_settle_cycles').value
        )
        forward_distance = float(
            self.manager.get_parameter('rotate_forward_distance').value
        )
        started = time.monotonic()
        try:
            if max_speed <= 0.0 or rate <= 0.0:
                result.message = 'rotate_speed must be positive'
                handle.abort()
                return result
            if forward_distance < 0.0 or not math.isfinite(forward_distance):
                result.message = (
                    'rotate_forward_distance must be finite and non-negative'
                )
                handle.abort()
                return result
            previous_yaw = self.manager.get_odometry_yaw()
            if previous_yaw is None:
                result.message = 'Fresh odometry is unavailable'
                handle.abort()
                return result
            accumulated_angle = 0.0
            settled_cycles = 0
            while time.monotonic() - started < timeout:
                if handle.is_cancel_requested:
                    result.message = 'Rotation cancelled'
                    handle.canceled()
                    return result
                current_yaw = self.manager.get_odometry_yaw()
                if current_yaw is None:
                    result.message = 'Odometry was lost during rotation'
                    handle.abort()
                    return result
                accumulated_angle += wrap_angle(current_yaw - previous_yaw)
                previous_yaw = current_yaw
                error = angle - accumulated_angle
                if abs(error) <= tolerance:
                    settled_cycles += 1
                    self.manager.stop()
                else:
                    settled_cycles = 0
                    command = max(-max_speed, min(max_speed, kp * error))
                    if abs(command) < min_speed:
                        command = math.copysign(min_speed, error)
                    self.manager.publish_velocity(wz=command)
                feedback = RotateBlind.Feedback()
                feedback.remaining_angle = float(error)
                handle.publish_feedback(feedback)
                if settled_cycles >= required_cycles:
                    self.manager.stop()
                    if forward_distance > 0.0:
                        failure = self._move_forward(
                            handle, forward_distance, rate, timeout
                        )
                        if failure is not None:
                            result.message = failure
                            if handle.is_cancel_requested:
                                handle.canceled()
                            else:
                                handle.abort()
                            return result
                    result.success = True
                    result.message = (
                        'Rotation and forward movement complete'
                        if forward_distance > 0.0
                        else 'Odometry-tracked rotation complete'
                    )
                    handle.succeed()
                    return result
                time.sleep(1.0 / rate)
            result.message = 'Rotation timed out before reaching its target'
            handle.abort()
            return result
        finally:
            self.manager.stop()
            self.manager.release_motion()

    def _move_forward(self, handle, distance, rate, timeout):
        speed = abs(float(
            self.manager.get_parameter('max_linear_velocity').value
        ))
        tolerance = abs(float(
            self.manager.get_parameter('position_tolerance').value
        ))
        if speed <= 0.0:
            return 'max_linear_velocity must be positive for forward movement'

        start_pose = self.manager.get_odometry_pose()
        if start_pose is None:
            return 'Fresh odometry is unavailable before forward movement'
        start_x, start_y, heading = start_pose
        started = time.monotonic()

        while time.monotonic() - started < timeout:
            if handle.is_cancel_requested:
                return 'Forward movement cancelled'
            pose = self.manager.get_odometry_pose()
            if pose is None:
                return 'Odometry was lost during forward movement'
            x, y, _ = pose
            traveled = (
                (x - start_x) * math.cos(heading)
                + (y - start_y) * math.sin(heading)
            )
            remaining = distance - traveled
            if remaining <= tolerance:
                self.manager.stop()
                return None
            self.manager.publish_velocity(vx=min(speed, remaining * rate))
            time.sleep(1.0 / rate)

        return 'Forward movement timed out before reaching its target'
