import math
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
        parcel_seen = False
        last_seen_time = None
        centered_cycles = 0
        self.manager.set_vision_active(True)
        try:
            timeout = float(self.manager.get_parameter('search_timeout').value)
            speed = float(self.manager.get_parameter('search_speed').value)
            center_kp = float(
                self.manager.get_parameter('search_center_kp').value
            )
            center_tolerance = float(
                self.manager.get_parameter('search_center_tolerance').value
            )
            required_cycles = int(
                self.manager.get_parameter('search_center_settle_cycles').value
            )
            reacquire_timeout = float(
                self.manager.get_parameter('search_reacquire_timeout').value
            )
            while time.monotonic() - started < timeout:
                if handle.is_cancel_requested:
                    result.message = 'Search cancelled'
                    handle.canceled()
                    return result
                item = self.manager.get_detection(handle.request.marker_id)
                if item is None:
                    if parcel_seen:
                        self.manager.stop()
                        centered_cycles = 0
                        if time.monotonic() - last_seen_time > reacquire_timeout:
                            result.message = 'Parcel was lost while centering'
                            handle.abort()
                            return result
                    else:
                        self.manager.publish_velocity(vx=speed)
                    detected = False
                else:
                    parcel_seen = True
                    last_seen_time = time.monotonic()
                    detection, _header, _received = item
                    position = detection.pose.position
                    horizontal_error = math.atan2(position.x, position.z)
                    centered = abs(horizontal_error) <= center_tolerance
                    centered_cycles = centered_cycles + 1 if centered else 0
                    if centered_cycles >= required_cycles:
                        result.success = True
                        result.message = 'Parcel centered in camera image'
                        handle.succeed()
                        return result
                    center_velocity = 0.0 if centered else center_kp * horizontal_error
                    center_velocity = max(-speed, min(speed, center_velocity))
                    self.manager.publish_velocity(vx=center_velocity)
                    detected = True
                feedback = SearchParcel.Feedback()
                feedback.detected = detected
                feedback.elapsed_time = float(time.monotonic() - started)
                handle.publish_feedback(feedback)
                time.sleep(0.05)
            result.message = 'Search timed out'
            handle.abort()
            return result
        finally:
            self.manager.stop()
            self.manager.set_vision_active(False)
            self.manager.release_motion()
