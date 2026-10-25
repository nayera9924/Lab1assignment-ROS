from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config1 = os.path.join(
      get_package_share_directory('node_pkg'),
      'config',
      'turtlesim.yaml'
      )
    config2 = os.path.join(
      get_package_share_directory('node_pkg'),
      'config',
      'GoToPose.yaml'
      )


    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen',
            parameters=[config1]
        ),
        Node(
            package='node_pkg',
            executable='speed_control_node',
            name='speed_control_node',

        ),
        
        Node(
            package='node_pkg',
            executable='go_to_pose_server',
            name='go_to_pose_server',
            output='screen',
            parameters=[config2]
        ),

        Node(
            package='node_pkg',
            executable='square_action_server',
            name='square_action_server',
            output='screen'
        )
    ])