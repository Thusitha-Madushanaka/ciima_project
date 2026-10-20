import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('goal_pose_controller'),
        'config',
        'goal_pose_controller.yaml'
    )

    goal_pose_controller = Node(
        package='goal_pose_controller',
        executable='goal_pose_controller',
        name='goal_pose_controller',
        output='screen',
        parameters=[
            config_file
        ]
    )

    return LaunchDescription([
        goal_pose_controller
    ])