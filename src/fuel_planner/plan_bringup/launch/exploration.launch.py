from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    odom_topic = LaunchConfiguration("odom_topic")
    sensor_pose_topic = LaunchConfiguration("sensor_pose_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    launch_waypoint_generator = LaunchConfiguration("launch_waypoint_generator")
    waypoint_goal_topic = LaunchConfiguration("waypoint_goal_topic")
    waypoint_traj_trigger = LaunchConfiguration("waypoint_traj_trigger")
    waypoint_type = LaunchConfiguration("waypoint_type")
    
    default_params = PathJoinSubstitution([
        FindPackageShare("exploration_manager"),
        "config",
        "exploration.yaml",
    ])

    lkh_tsp_solver_share = FindPackageShare("lkh_tsp_solver")
    tsp_dir = PathJoinSubstitution([lkh_tsp_solver_share, "resource"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML file providing exploration_node parameters.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odom_world",
            description="Odometry topic supplying UAV pose/velocity.",
        ),
        DeclareLaunchArgument(
            "sensor_pose_topic",
            default_value="/pcl_render_node/sensor_pose",
            description="Pose topic from the onboard depth sensor.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/pcl_render_node/depth",
            description="Depth image topic consumed by map_ros.",
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/pcl_render_node/cloud",
            description="Point cloud topic consumed by map_ros.",
        ),
        DeclareLaunchArgument(
            "launch_waypoint_generator",
            default_value="true",
            description="Set true to start waypoint_generator alongside the planner.",
        ),
        DeclareLaunchArgument(
            "waypoint_goal_topic",
            default_value="/move_base_simple/goal",
            description="Goal topic listened by waypoint_generator when enabled.",
        ),
        DeclareLaunchArgument(
            "waypoint_traj_trigger",
            default_value="/traj_start_trigger",
            description="Trigger topic used by waypoint_generator for trajectory starts.",
        ),
        DeclareLaunchArgument(
            "waypoint_type",
            default_value="point",
            description="Configures the waypoint_generator node behavior.",
        ),
        Node(
            package="exploration_manager",
            executable="exploration_node",
            name="exploration_node",
            parameters=[
                params_file,
                {"exploration/tsp_dir": tsp_dir},
            ],
            remappings=[
                ("/odom_world", odom_topic),
                ("/map_ros/pose", sensor_pose_topic),
                ("/map_ros/depth", depth_topic),
                ("/map_ros/cloud", cloud_topic),
            ],
            output="screen",
        ),
        Node(
            package="waypoint_generator",
            executable="waypoint_generator",
            name="waypoint_generator",
            condition=IfCondition(launch_waypoint_generator),
            remappings=[
                ("odom", odom_topic),
                ("goal", waypoint_goal_topic),
                ("traj_start_trigger", waypoint_traj_trigger),
                ("waypoints", "/waypoint_generator/waypoints"),
            ],
            parameters=[{"waypoint_type": waypoint_type}],
            output="screen",
        ),
    ])
