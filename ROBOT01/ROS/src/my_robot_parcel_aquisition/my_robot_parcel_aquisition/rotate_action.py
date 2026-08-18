import math
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from my_robot_parcel_aquisition.action import RotateBlind


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
        speed = abs(float(self.manager.get_parameter('rotate_speed').value))
        started = time.monotonic()
        try:
            if speed <= 0.0:
                result.message = 'rotate_speed must be positive'
                handle.abort()
                return result
            duration = abs(angle) / speed
            command = math.copysign(speed, angle)
            while time.monotonic() - started < duration:
                if handle.is_cancel_requested:
                    result.message = 'Rotation cancelled'
                    handle.canceled()
                    return result
                self.manager.publish_velocity(wz=command)
                elapsed = time.monotonic() - started
                feedback = RotateBlind.Feedback()
                feedback.remaining_angle = float(math.copysign(max(0.0, abs(angle) - speed * elapsed), angle))
                handle.publish_feedback(feedback)
                time.sleep(0.05)
            result.success = True
            result.message = 'Blind rotation complete'
            handle.succeed()
            return result
        finally:
            self.manager.stop()
            self.manager.release_motion()
