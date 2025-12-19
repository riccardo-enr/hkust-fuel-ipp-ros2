from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  params_file = LaunchConfiguration("params_file")
  planner_mode = LaunchConfiguration("planner_mode")
  odom_topic = LaunchConfiguration("odom_topic")
  sensor_pose_topic = LaunchConfiguration("sensor_pose_topic")
  depth_topic = LaunchConfiguration("depth_topic")
  cloud_topic = LaunchConfiguration("cloud_topic")
  launch_waypoint_generator = LaunchConfiguration("launch_waypoint_generator")
  waypoint_goal_topic = LaunchConfiguration("waypoint_goal_topic")
  waypoint_traj_trigger = LaunchConfiguration("waypoint_traj_trigger")
  waypoint_type = LaunchConfiguration("waypoint_type")

  default_params = PathJoinSubstitution([
      FindPackageShare("plan_manage"),
      "config",
      "fast_planner.yaml",
  ])

  return LaunchDescription([
      DeclareLaunchArgument(
          "params_file",
          default_value=default_params,
          description="YAML file providing fast_planner_node parameters.",
      ),
      DeclareLaunchArgument(
          "planner_mode",
          default_value="1",
          description="Planner selector: 1=kino_replan, 2=topo_replan, 3=local_explore.",
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
          description="Point cloud topic consumed by map_ros (leave empty to disable).",
      ),
      DeclareLaunchArgument(
          "launch_waypoint_generator",
          default_value="false",
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
          default_value="manual-lonely-waypoint",
          description="Configures the waypoint_generator node behavior.",
      ),
      Node(
          package="plan_manage",
          executable="fast_planner_node",
          name="fast_planner_node",
          parameters=[
              params_file,
              {
                  "planner_node/planner": planner_mode,
              },
          ],
          remappings=[
              ("/odom_world", odom_topic),
              ("/map_ros/pose", sensor_pose_topic),
              ("/map_ros/depth", depth_topic),
              ("/map_ros/cloud", cloud_topic),
              ("/waypoint_generator/waypoints", "/waypoints"),
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
          ],
          parameters=[{"waypoint_type": waypoint_type}],
          output="screen",
      ),
  ])
