
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from interfaces_pkg.action import GoToPose, Square
from interfaces_pkg.msg import TargetPose


class SquareActionServer(Node):

    def __init__(self):
        super().__init__('square_action_server')

        self.cb_group = ReentrantCallbackGroup()

        self.square_server = ActionServer(
            self,
            Square,
            'draw_square',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.go_to_pose_client = ActionClient(
            self,
            GoToPose,
            'go_to_pose',
            callback_group=self.cb_group
        )

        self.reset_client = self.create_client(
            Trigger,
            '/reset_turtle',
            callback_group=self.cb_group
        )

        self.get_logger().info('Square Action Server is ready.')

    def goal_callback(self, goal_request):
        side = goal_request.side_length

        if not math.isfinite(side) or side <= 0.0 or side > 5.0:
            self.get_logger().warn('Rejected: side length must be > 0 and <= 5.')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Square cancellation requested.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = Square.Result()
        feedback = Square.Feedback()

        side = float(goal_handle.request.side_length)
        feedback.sides_completed = 0

        try:
            # Wait for the Reset service and GoToPose action server.
            if not self.reset_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('Reset service /reset_turtle is unavailable.')

            if not self.go_to_pose_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError('GoToPose action server is unavailable.')

            # Reset the turtle before drawing.
            reset_future = self.reset_client.call_async(Trigger.Request())
            while rclpy.ok() and not reset_future.done():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Square drawing canceled.'
                    return result
                time.sleep(0.02)

            reset_response = reset_future.result()
            if reset_response is None or not reset_response.success:
                raise RuntimeError('Failed to reset the turtle.')

            # Start from the standard turtlesim home position.
            start_x = 5.544445
            start_y = 5.544445

            corners = [
                (start_x + side, start_y),
                (start_x + side, start_y + side),
                (start_x, start_y + side),
                (start_x, start_y),
            ]

            for index, (x, y) in enumerate(corners):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Square drawing canceled.'
                    return result

                request = GoToPose.Goal()
                request.target_pose = TargetPose()
                request.target_pose.x = float(x)
                request.target_pose.y = float(y)

                # Face the next corner, then return to the starting heading.
                if index == 0:
                    request.target_pose.yaw = 0.0
                elif index == 1:
                    request.target_pose.yaw = math.pi / 2.0
                elif index == 2:
                    request.target_pose.yaw = math.pi
                else:
                    request.target_pose.yaw = -math.pi / 2.0

                send_future = self.go_to_pose_client.send_goal_async(request)

                while rclpy.ok() and not send_future.done():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = 'Square drawing canceled.'
                        return result
                    time.sleep(0.02)

                go_handle = send_future.result()
                if go_handle is None or not go_handle.accepted:
                    raise RuntimeError(f'GoToPose rejected side {index + 1}.')

                result_future = go_handle.get_result_async()
                while rclpy.ok() and not result_future.done():
                    if goal_handle.is_cancel_requested:
                        go_handle.cancel_goal_async()
                        goal_handle.canceled()
                        result.success = False
                        result.message = 'Square drawing canceled.'
                        return result
                    time.sleep(0.02)

                go_result = result_future.result()
                if go_result is None or not go_result.result.success:
                    raise RuntimeError(f'GoToPose failed on side {index + 1}.')

                feedback.sides_completed = index + 1
                goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result.success = True
            result.message = 'Square drawn successfully using four GoToPose calls.'
            self.get_logger().info(result.message)
            return result

        except Exception as exc:
            result.success = False
            result.message = str(exc)
            goal_handle.abort()
            self.get_logger().error(result.message)
            return result


def main(args=None):
    rclpy.init(args=args)
    node = SquareActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
