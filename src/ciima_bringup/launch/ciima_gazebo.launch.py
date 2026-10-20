import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch.substitutions import Command


def generate_launch_description():

    # Locate existing ROS 2 packages
    gazebo_ros_path = get_package_share_directory("gazebo_ros")
    description_path = get_package_share_directory("ciima_description")

    # Use your existing robot Xacro
    xacro_file = os.path.join(
        description_path,
        "xacro",
        "ciima_description.urdf.xacro"
    )

    # Convert Xacro to URDF
    robot_description = {
        "robot_description": Command(["xacro ", xacro_file])
    }

    # Launch Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_path,
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    # Publish robot description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "ciima_robot",
            "-x", "0",
            "-y", "0",
            "-z", "0.5"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
