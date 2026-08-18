import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from my_robot_parcel_aquisition.action import SearchParcel


class SearchAction:
    def __init__(self, manager):
        self.manager = manager
        self.server = ActionServer(
            manager, SearchParcel, 'search_parcel', self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=manager.callback_group,
        )

    @staticmethod
    def goal(request):
        return GoalResponse.ACCEPT if request.marker_id >= 0 else GoalResponse.REJECT

    @staticmethod
    def cancel(_handle):
        return CancelResponse.ACCEPT

    def execute(self, handle):
        result = SearchParcel.Result()
        if not self.manager.try_claim_motion():
            result.message = 'Another acquisition action owns velocity control'
            handle.abort()
            return result
        started = time.monotonic()
        try:
            timeout = float(self.manager.get_parameter('search_timeout').value)
            speed = float(self.manager.get_parameter('search_speed').value)
            while time.monotonic() - started < timeout:
                if handle.is_cancel_requested:
                    result.message = 'Search cancelled'
                    handle.canceled()
                    return result
                if self.manager.get_detection(handle.request.marker_id) is not None:
                    result.success = True
                    result.message = 'Parcel detected'
                    handle.succeed()
                    return result
                self.manager.publish_velocity(vx=speed)
                feedback = SearchParcel.Feedback()
                feedback.detected = False
                feedback.elapsed_time = float(time.monotonic() - started)
                handle.publish_feedback(feedback)
                time.sleep(0.05)
            result.message = 'Search timed out'
            handle.abort()
            return result
        finally:
            self.manager.stop()
            self.manager.release_motion()
