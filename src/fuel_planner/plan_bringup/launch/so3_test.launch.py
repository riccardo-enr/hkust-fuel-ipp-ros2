import os
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    plan_bringup_dir = get_package_share_directory('plan_bringup')
    plan_manage_dir = get_package_share_directory('plan_manage')

    # Include the main simulator launch file
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plan_bringup_dir, 'launch', 'simulator.launch.py')
        ),
        launch_arguments={'map_type': 'empty_world'}.items()
    )

    # Launch RViz2 with the clean configuration
    rviz_config_path = os.path.join(plan_manage_dir, 'config', 'traj_viz_dev.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Command to publish a test position after 8 seconds
    # Note: We use 'ros2 topic pub' with 'execute_process'
    test_command = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/planning/pos_cmd',
            'quadrotor_msgs/msg/PositionCommand',
            '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "world"}, position: {x: -18.0, y: -1.0, z: 2.0}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0, kx: [5.7, 5.7, 6.2], kv: [3.4, 3.4, 4.0], trajectory_flag: 1}'
        ],
        output='screen'
    )

    delayed_test_command = TimerAction(
        period=8.0,
        actions=[test_command]
    )

    return LaunchDescription([
        simulator_launch,
        rviz_node,
        delayed_test_command
    ])
