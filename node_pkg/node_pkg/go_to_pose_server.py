

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty as EmptySrv, Trigger
from rclpy.callback_groups import ReentrantCallbackGroup

from interfaces_pkg.action import GoToPose


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GoToPoseServer(Node):

    def __init__(self):
        super().__init__('go_to_pose_server')

        self.cb_group = ReentrantCallbackGroup()
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

        #subscriber and publisher
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

        #Action server
        self.action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Clients
        self.clear_client = self.create_client(EmptySrv, '/clear', callback_group=self.cb_group)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=self.cb_group)
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen', callback_group=self.cb_group)

        # Reset service server
        self.reset_service = self.create_service(
            Trigger,
            '/reset_turtle',
            self.reset_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('GoToPose Action Server is ready.')

    def pose_callback(self, msg):
        self.pose = msg

    def goal_callback(self, goal_request):
        target = goal_request.target_pose

        if not all(math.isfinite(v) for v in
                   (target.x, target.y, target.yaw)):
            return GoalResponse.REJECT

        if not (0.0 <= target.x <= 11.0 and
                0.0 <= target.y <= 11.0):
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received.')
        return CancelResponse.ACCEPT

    def stop_turtle(self):
        self.velocity_publisher.publish(Twist())

    def reset_callback(self, request, response):
        self.get_logger().info('Resetting turtlesim environment...')

        # 1.stop turtle
        self.stop_turtle()

        # 2.stop drawing
        pen_req = SetPen.Request()
        pen_req.off = 1
        self.set_pen_client.call_async(pen_req)

        # 3.clear
        clear_req = EmptySrv.Request()
        self.clear_client.call_async(clear_req)

        # 4.return turtle to origin
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = 5.544445
        teleport_req.y = 5.544445
        teleport_req.theta = 0.0
        self.teleport_client.call_async(teleport_req)   #to can move after reset
        time.sleep(0.05)
        
        # future = self.teleport_client.call_async(teleport_req)
        # rclpy.spin_until_future_complete(self, future)

        # 5.back to drawing
        pen_req.off = 0
        self.set_pen_client.call_async(pen_req)

        response.success = True
        response.message = 'Turtle reset successfully to home position!'
        self.get_logger().info(response.message)
        return response

    def execute_callback(self, goal_handle):
        result = GoToPose.Result()
        feedback = GoToPose.Feedback()
        target = goal_handle.request.target_pose

        try:
            while rclpy.ok() and self.pose is None:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled.'
                    return result
                time.sleep(0.05)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.stop_turtle()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled.'
                    return result

                current = self.pose
                if current is None:
                    time.sleep(0.05)
                    continue

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
                time.sleep(0.05)

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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_turtle()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

