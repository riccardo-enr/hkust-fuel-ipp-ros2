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
    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    box_min_x = LaunchConfiguration("box_min_x")
    box_min_y = LaunchConfiguration("box_min_y")
    box_min_z = LaunchConfiguration("box_min_z")
    box_max_x = LaunchConfiguration("box_max_x")
    box_max_y = LaunchConfiguration("box_max_y")
    box_max_z = LaunchConfiguration("box_max_z")
    
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
        DeclareLaunchArgument(
            "map_size_x",
            default_value="40.0",
            description="Map size in X direction.",
        ),
        DeclareLaunchArgument(
            "map_size_y",
            default_value="20.0",
            description="Map size in Y direction.",
        ),
        DeclareLaunchArgument(
            "map_size_z",
            default_value="5.0",
            description="Map size in Z direction.",
        ),
        DeclareLaunchArgument("box_min_x", default_value="-20.0"),
        DeclareLaunchArgument("box_min_y", default_value="-10.0"),
        DeclareLaunchArgument("box_min_z", default_value="0.0"),
        DeclareLaunchArgument("box_max_x", default_value="20.0"),
        DeclareLaunchArgument("box_max_y", default_value="10.0"),
        DeclareLaunchArgument("box_max_z", default_value="2.5"),
        Node(
            package="exploration_manager",
            executable="exploration_node",
            name="exploration_node",
            parameters=[
                params_file,
                {
                    "exploration/tsp_dir": tsp_dir,
                    "sdf_map/map_size_x": map_size_x,
                    "sdf_map/map_size_y": map_size_y,
                    "sdf_map/map_size_z": map_size_z,
                    "sdf_map/box_min_x": box_min_x,
                    "sdf_map/box_min_y": box_min_y,
                    "sdf_map/box_min_z": box_min_z,
                    "sdf_map/box_max_x": box_max_x,
                    "sdf_map/box_max_y": box_max_y,
                    "sdf_map/box_max_z": box_max_z,
                },
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
