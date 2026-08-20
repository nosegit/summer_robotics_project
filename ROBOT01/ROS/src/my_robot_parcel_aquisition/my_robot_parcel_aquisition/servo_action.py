import math
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.time import Time
from my_robot_parcel_aquisition.action import ServoParcel
from my_robot_parcel_aquisition.geometry import (
    desaturate,
    parallel_yaw_error,
    transform_detection,
)


class ServoAction:
    def __init__(self, manager):
        self.manager = manager
        self.server = ActionServer(
            manager, ServoParcel, 'servo_parcel', self.execute,
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
        result = ServoParcel.Result()
        if not self.manager.try_claim_motion():
            result.message = 'Another acquisition action owns velocity control'
            handle.abort()
            return result
        started = time.monotonic()
        settled = 0
        parcel_seen = False
        try:
            rate = float(self.manager.get_parameter('servo_rate').value)
            timeout = float(self.manager.get_parameter('servo_timeout').value)
            target_x = float(self.manager.get_parameter('target_x').value)
            target_y = float(self.manager.get_parameter('target_y').value)
            target_yaw = float(self.manager.get_parameter('target_yaw').value)
            pos_tol = float(self.manager.get_parameter('position_tolerance').value)
            yaw_tol = float(self.manager.get_parameter('yaw_tolerance').value)
            required = int(self.manager.get_parameter('settle_cycles').value)
            while time.monotonic() - started < timeout:
                if handle.is_cancel_requested:
                    result.message = 'Servo cancelled'
                    handle.canceled()
                    return result
                item = self.manager.get_detection(handle.request.marker_id)
                if item is None:
                    self.manager.stop()
                    settled = 0
                    if parcel_seen:
                        result.message = 'Parcel detection lost for longer than the freshness timeout'
                        handle.abort()
                        return result
                    time.sleep(1.0 / rate)
                    continue
                parcel_seen = True
                detection, header, _received = item
                try:
                    transform = self.manager.tf_buffer.lookup_transform(
                        self.manager.get_parameter('base_frame').value,
                        header.frame_id, Time(),
                    )
                except Exception as error:
                    self.manager.get_logger().warn(
                        f'Cannot transform parcel detection: {error}',
                        throttle_duration_sec=1.0,
                    )
                    self.manager.stop()
                    time.sleep(1.0 / rate)
                    continue
                center, normal = transform_detection(detection, transform)
                x_error = center[0] - target_x
                y_error = center[1] - target_y
                face_yaw = math.atan2(normal[1], normal[0])
                yaw_error = parallel_yaw_error(face_yaw, target_yaw)
                x_aligned = abs(x_error) <= pos_tol
                y_aligned = abs(y_error) <= pos_tol
                yaw_aligned = abs(yaw_error) <= yaw_tol
                vx = (
                    0.0 if x_aligned
                    else float(self.manager.get_parameter('kp_x').value) * x_error
                )
                vy = (
                    0.0 if y_aligned
                    else float(self.manager.get_parameter('kp_y').value) * y_error
                )
                wz = (
                    0.0 if yaw_aligned
                    else float(self.manager.get_parameter('kp_yaw').value) * yaw_error
                )
                vx, vy, wz = desaturate(
                    vx, vy, wz,
                    float(self.manager.get_parameter('max_linear_velocity').value),
                    float(self.manager.get_parameter('max_angular_velocity').value),
                )
                self.manager.publish_velocity(vx, vy, wz)
                feedback = ServoParcel.Feedback()
                feedback.x_error = float(x_error)
                feedback.y_error = float(y_error)
                feedback.yaw_error = float(yaw_error)
                handle.publish_feedback(feedback)
                aligned = x_aligned and y_aligned and yaw_aligned
                settled = settled + 1 if aligned else 0
                if settled >= required:
                    result.success = True
                    result.message = 'Parcel alignment reached'
                    handle.succeed()
                    return result
                time.sleep(1.0 / rate)
            result.message = 'Servo timed out or the parcel remained unavailable'
            handle.abort()
            return result
        finally:
            self.manager.stop()
            self.manager.release_motion()
