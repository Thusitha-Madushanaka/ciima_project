import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import tempfile
import xacro

def generate_launch_description():

    # Configuration files
    xacro_file = os.path.join(get_package_share_directory('ciima_description'), 'xacro', 'ciima_description.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro', ' ', xacro_file, ' ', 'prefix:=', 'fake_robot',]),
                                     value_type=str)


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("bringup"),
            "configs",
            "controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    custom_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "custom_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen"
    )

    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ]
    )

    rviz_config_file = os.path.join( get_package_share_directory('bringup'), 'configs','config.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    goal_pose_config = os.path.join(
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
            goal_pose_config
        ]   
    )


    return LaunchDescription([
        robot_state_publisher,

        control_node,
        custom_controller,
        joint_state_broadcaster_spawner,
        goal_pose_controller,

        rviz_node,
    ])