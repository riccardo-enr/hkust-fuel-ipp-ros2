from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    odom_topic = LaunchConfiguration("odom_topic")
    command_topic = LaunchConfiguration("command_topic")
    use_backup = LaunchConfiguration("use_backup")

    default_params = PathJoinSubstitution([
        FindPackageShare("plan_manage"),
        "config",
        "traj_server.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML file providing traj_server parameters.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odom_world",
            description="Odometry topic supplying world-frame state.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="/position_cmd",
            description="Topic used to publish quadrotor position commands.",
        ),
        DeclareLaunchArgument(
            "use_backup",
            default_value="false",
            description="Set true to run the simpler traj_server_backup node.",
        ),
        Node(
            package="plan_manage",
            executable="traj_server",
            name="traj_server",
            condition=UnlessCondition(use_backup),
            parameters=[
                params_file,
                {
                    "traj_server.odom_topic": odom_topic,
                    "traj_server.command_topic": command_topic,
                },
            ],
            output="screen",
        ),
        Node(
            package="plan_manage",
            executable="traj_server_backup",
            name="traj_server_backup",
            condition=IfCondition(use_backup),
            parameters=[
                params_file,
                {
                    "traj_server.odom_topic": odom_topic,
                    "traj_server.command_topic": command_topic,
                },
            ],
            output="screen",
        ),
    ])
