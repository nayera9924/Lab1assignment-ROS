
import math
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from interfaces_pkg.action import GoToPose


def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('position_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.1)

        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )
        self.angle_tolerance = float(
            self.get_parameter('angle_tolerance').value
        )

        self.pose = None

        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.velocity_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(
            'GoToPose Action Server is ready.'
        )

    def pose_callback(self, msg):
        self.pose = msg

    def goal_callback(self, goal_request):
        target = goal_request.target_pose

        if not all(math.isfinite(v) for v in
                   (target.x, target.y, target.yaw)):
            self.get_logger().warning(
                'Rejected goal: target coordinates must be finite.'
            )
            return GoalResponse.REJECT

        if not (0.0 <= target.x <= 11.0 and
                0.0 <= target.y <= 11.0):
            self.get_logger().warning(
                'Rejected goal: target is outside the turtlesim area.'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepted goal: x={target.x:.2f}, '
            f'y={target.y:.2f}, yaw={target.yaw:.2f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received.')
        return CancelResponse.ACCEPT

    def stop_turtle(self):
        self.velocity_publisher.publish(Twist())

    async def execute_callback(self, goal_handle):
        result = GoToPose.Result()
        feedback = GoToPose.Feedback()
        target = goal_handle.request.target_pose

        try:
            # Wait until the first pose message arrives.
            while rclpy.ok() and self.pose is None:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled.'
                    return result
                await asyncio.sleep(0.05)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.stop_turtle()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled.'
                    return result

                current = self.pose
                dx = target.x - current.x
                dy = target.y - current.y
                distance = math.hypot(dx, dy)

                feedback.distance_to_goal = float(distance)
                goal_handle.publish_feedback(feedback)

                cmd = Twist()

                if distance > self.position_tolerance:
                    desired_heading = math.atan2(dy, dx)
                    heading_error = normalize_angle(
                        desired_heading - current.theta
                    )

                    # Turn toward the target before moving forward.
                    if abs(heading_error) > 0.15:
                        cmd.angular.z = max(
                            -self.max_angular_speed,
                            min(
                                self.max_angular_speed,
                                4.0 * heading_error
                            )
                        )
                    else:
                        cmd.linear.x = min(
                            self.max_linear_speed,
                            1.5 * distance
                        )
                        cmd.angular.z = max(
                            -self.max_angular_speed,
                            min(
                                self.max_angular_speed,
                                4.0 * heading_error
                            )
                        )
                else:
                    # Position reached: rotate to the requested yaw.
                    yaw_error = normalize_angle(
                        target.yaw - current.theta
                    )

                    if abs(yaw_error) <= self.angle_tolerance:
                        self.stop_turtle()
                        goal_handle.succeed()
                        result.success = True
                        result.message = 'Target pose reached successfully.'
                        self.get_logger().info(result.message)
                        return result

                    cmd.angular.z = max(
                        -self.max_angular_speed,
                        min(
                            self.max_angular_speed,
                            3.0 * yaw_error
                        )
                    )

                self.velocity_publisher.publish(cmd)
                await asyncio.sleep(0.05)

            self.stop_turtle()
            result.success = False
            result.message = 'ROS 2 stopped before reaching the goal.'
            goal_handle.abort()
            return result

        except Exception as exc:
            self.stop_turtle()
            result.success = False
            result.message = f'Action failed: {exc}'
            goal_handle.abort()
            self.get_logger().error(result.message)
            return result

        finally:
            self.stop_turtle()


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_turtle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
