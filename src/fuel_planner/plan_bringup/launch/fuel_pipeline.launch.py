import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
  plan_bringup_dir = get_package_share_directory("plan_bringup")

  map_type = LaunchConfiguration("map_type")
  world = LaunchConfiguration("world")
  odom_topic = LaunchConfiguration("odom_topic")
  command_topic = LaunchConfiguration("command_topic")
  planner_mode = LaunchConfiguration("planner_mode")
  sensor_pose_topic = LaunchConfiguration("sensor_pose_topic")
  depth_topic = LaunchConfiguration("depth_topic")
  cloud_topic = LaunchConfiguration("cloud_topic")
  launch_waypoint_generator = LaunchConfiguration("launch_waypoint_generator")
  waypoint_goal_topic = LaunchConfiguration("waypoint_goal_topic")
  waypoint_traj_trigger = LaunchConfiguration("waypoint_traj_trigger")
  waypoint_type = LaunchConfiguration("waypoint_type")
  use_backup = LaunchConfiguration("use_backup")
  launch_rviz = LaunchConfiguration("launch_rviz")
  rviz_config = LaunchConfiguration("rviz_config")
  publish_simulator_tf = LaunchConfiguration("publish_simulator_tf")
  traj_server_params_file = LaunchConfiguration("traj_server_params_file")
  fast_planner_params_file = LaunchConfiguration("fast_planner_params_file")
  exploration_params_file = LaunchConfiguration("exploration_params_file")
  init_x = LaunchConfiguration("init_x")
  init_y = LaunchConfiguration("init_y")
  init_z = LaunchConfiguration("init_z")
  exploration = LaunchConfiguration("exploration")
  controller_type = LaunchConfiguration("controller_type")

  # Use 'world' if provided, otherwise 'map_type'
  final_map_type = PythonExpression(["'", world, "' if '", world, "' != '' else '", map_type, "'"])

  plan_manage_dir = get_package_share_directory("plan_manage")
  exploration_manager_dir = get_package_share_directory("exploration_manager")

  simulator_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(plan_bringup_dir, "launch", "simulator.launch.py")
    ),
    launch_arguments={
      "map_type": final_map_type,
      "odometry_topic": odom_topic,
      "init_x": init_x,
      "init_y": init_y,
      "init_z": init_z,
      "controller_type": controller_type,
    }.items(),
  )

  traj_server_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(plan_bringup_dir, "launch", "traj_server.launch.py")
      ),
      launch_arguments={
          "odom_topic": odom_topic,
          "command_topic": command_topic,
          "use_backup": use_backup,
          "init_x": init_x,
          "init_y": init_y,
          "init_z": init_z,
      }.items(),
  )

  fast_planner_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(plan_bringup_dir, "launch", "fast_planner.launch.py")
      ),
      launch_arguments={
          "params_file": fast_planner_params_file,
          "planner_mode": planner_mode,
          "odom_topic": odom_topic,
          "sensor_pose_topic": sensor_pose_topic,
          "depth_topic": depth_topic,
          "cloud_topic": cloud_topic,
          "launch_waypoint_generator": launch_waypoint_generator,
          "waypoint_goal_topic": waypoint_goal_topic,
          "waypoint_traj_trigger": waypoint_traj_trigger,
          "waypoint_type": waypoint_type,
      }.items(),
      condition=UnlessCondition(exploration)
  )

  exploration_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(plan_bringup_dir, "launch", "exploration.launch.py")
      ),
      launch_arguments={
          "params_file": exploration_params_file,
          "odom_topic": odom_topic,
          "sensor_pose_topic": sensor_pose_topic,
          "depth_topic": depth_topic,
          "cloud_topic": cloud_topic,
          "launch_waypoint_generator": launch_waypoint_generator,
          "waypoint_goal_topic": waypoint_goal_topic,
          "waypoint_traj_trigger": waypoint_traj_trigger,
          "waypoint_type": waypoint_type,
      }.items(),
      condition=IfCondition(exploration)
  )

  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      condition=IfCondition(launch_rviz),
      arguments=["-d", rviz_config],
      output="screen",
  )

  simulator_tf = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="simulator_to_world_tf",
      condition=IfCondition(publish_simulator_tf),
      arguments=["0", "0", "0", "0", "0", "0", "1", "world", "simulator"],
      output="screen",
  )

  return LaunchDescription(
      [
          DeclareLaunchArgument(
              "map_type",
              default_value="random_forest",
              description="Scene generator type for the simulator: [random_forest, empty_world, office, office2, office3, pillar].",
          ),
          DeclareLaunchArgument(
              "world",
              default_value="",
              description="Alias for map_type. If set, it overrides map_type.",
          ),
          DeclareLaunchArgument(
              "odom_topic",
              default_value="/state_ukf/odom",
              description="Unified odometry topic fed to the planner stack.",
          ),
          DeclareLaunchArgument(
              "command_topic",
              default_value="/planning/pos_cmd",
              description="Topic that receives position commands from the planner.",
          ),
          DeclareLaunchArgument(
              "planner_mode",
              default_value="1",
              description="Planner mode selector: 1=kino_replan, 2=topo_replan, 3=local_explore.",
          ),
          DeclareLaunchArgument(
              "sensor_pose_topic",
              default_value="/pcl_render_node/sensor_pose",
              description="Pose topic produced by the point-cloud renderer.",
          ),
          DeclareLaunchArgument(
              "depth_topic",
              default_value="/pcl_render_node/depth",
              description="Depth image topic consumed by the planner.",
          ),
          DeclareLaunchArgument(
              "cloud_topic",
              default_value="/pcl_render_node/cloud",
              description="Point cloud topic consumed by the planner.",
          ),
          DeclareLaunchArgument(
              "launch_waypoint_generator",
              default_value="true",
              description="Start the waypoint_generator helper node alongside the planner.",
          ),
          DeclareLaunchArgument(
              "waypoint_goal_topic",
              default_value="/move_base_simple/goal",
              description="Topic watched by the waypoint_generator for manual goals.",
          ),
          DeclareLaunchArgument(
              "waypoint_traj_trigger",
              default_value="/traj_start_trigger",
              description="Trigger topic used to start waypoint trajectories.",
          ),
          DeclareLaunchArgument(
              "waypoint_type",
              default_value="manual-lonely-waypoint",
              description="Waypoint generation strategy.",
          ),
          DeclareLaunchArgument(
              "use_backup",
              default_value="false",
              description="Launch `traj_server_backup` instead of `traj_server` when true.",
          ),
          DeclareLaunchArgument(
              "launch_rviz",
              default_value="true",
              description="Launch RViz2 with a preconfigured view when true.",
          ),
          DeclareLaunchArgument(
              "rviz_config",
              default_value=os.path.join(plan_manage_dir, "config", "traj_viz_dev.rviz"),
              description="Path to the RViz2 config file.",
          ),
          DeclareLaunchArgument(
              "traj_server_params_file",
              default_value=os.path.join(plan_manage_dir, "config", "traj_server.yaml"),
              description="YAML file providing traj_server parameters.",
          ),
          DeclareLaunchArgument(
              "fast_planner_params_file",
              default_value=os.path.join(plan_manage_dir, "config", "fast_planner.yaml"),
              description="YAML file providing fast_planner parameters.",
          ),
          DeclareLaunchArgument(
              "exploration_params_file",
              default_value=os.path.join(exploration_manager_dir, "config", "exploration.yaml"),
              description="YAML file providing exploration_node parameters.",
          ),
          DeclareLaunchArgument(
              "publish_simulator_tf",
              default_value="true",
              description="Publish a world->simulator static transform for RViz.",
          ),
          DeclareLaunchArgument(
              "init_x",
              default_value="-19.99",
              description="Initial x position",
          ),
          DeclareLaunchArgument(
              "init_y",
              default_value="-1.0",
              description="Initial y position",
          ),
          DeclareLaunchArgument(
              "init_z",
              default_value="1.0",
              description="Initial z position",
          ),
          DeclareLaunchArgument(
              "exploration",
              default_value="false",
              description="Set true to run in autonomous exploration mode.",
          ),
          DeclareLaunchArgument(
              "controller_type",
              default_value="so3",
              description="Controller type: so3 or mppi",
          ),
          DeclareLaunchArgument(
              "exploration_threshold",
              default_value="75.0",
              description="Exploration percentage threshold to stop the simulation.",
          ),
          simulator_launch,
          SetLaunchConfiguration("params_file", traj_server_params_file),
          traj_server_launch,
          SetLaunchConfiguration("params_file", fast_planner_params_file),
          fast_planner_launch,
          exploration_launch,
          Node(
              package="plan_bringup",
              executable="exploration_supervisor",
              name="exploration_supervisor",
              parameters=[{"threshold": LaunchConfiguration("exploration_threshold")}],
              condition=IfCondition(exploration),
              output="screen",
          ),
          simulator_tf,
          rviz_node,
      ]
  )
