import rclpy
from rclpy.executors import MultiThreadedExecutor
from my_robot_parcel_aquisition.manager import AcquisitionManager
from my_robot_parcel_aquisition.rotate_action import RotateAction
from my_robot_parcel_aquisition.search_action import SearchAction
from my_robot_parcel_aquisition.servo_action import ServoAction


def main(args=None):
    rclpy.init(args=args)
    manager = AcquisitionManager()
    actions = (SearchAction(manager), ServoAction(manager), RotateAction(manager))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(manager)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop()
        for action in actions:
            action.server.destroy()
        executor.shutdown()
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
