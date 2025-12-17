from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    traj_marker_topic = LaunchConfiguration("traj_marker_topic")
    state_marker_topic = LaunchConfiguration("state_marker_topic")
    command_topic = LaunchConfiguration("command_topic")
    frame_id = LaunchConfiguration("frame_id")
    command_rate_hz = LaunchConfiguration("command_rate_hz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution([
        FindPackageShare("poly_traj"),
        "config",
        "traj_generator_sim.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/uwb_vicon_odom",
            description="Odometry topic driving the trajectory generator.",
        ),
        DeclareLaunchArgument(
            "traj_marker_topic",
            default_value="/traj_generator/traj_vis",
            description="Marker topic used to visualize the full trajectory.",
        ),
        DeclareLaunchArgument(
            "state_marker_topic",
            default_value="/traj_generator/cmd_vis",
            description="Marker topic for instantaneous state vectors.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="/drone_commander/onboard_command",
            description="Drone command topic output.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="world",
            description="Frame id applied to visualization markers.",
        ),
        DeclareLaunchArgument(
            "command_rate_hz",
            default_value="100.0",
            description="Publishing rate for onboard commands.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Pass true to use /clock from simulation.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to an optional YAML file with parameter overrides.",
        ),
        Node(
            package="poly_traj",
            executable="traj_generator",
            output="screen",
            parameters=[
                params_file,
                {
                    "odom_topic": odom_topic,
                    "traj_marker_topic": traj_marker_topic,
                    "state_marker_topic": state_marker_topic,
                    "command_topic": command_topic,
                    "frame_id": frame_id,
                    "command_rate_hz": command_rate_hz,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
