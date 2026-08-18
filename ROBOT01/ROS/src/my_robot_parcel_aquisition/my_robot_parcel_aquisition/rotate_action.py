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
        started = time.monotonic()
        try:
            if max_speed <= 0.0 or rate <= 0.0:
                result.message = 'rotate_speed must be positive'
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
                    result.success = True
                    result.message = 'Odometry-tracked rotation complete'
                    handle.succeed()
                    return result
                time.sleep(1.0 / rate)
            result.message = 'Rotation timed out before reaching its target'
            handle.abort()
            return result
        finally:
            self.manager.stop()
            self.manager.release_motion()
