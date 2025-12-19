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

    # Circle Publisher Node to publish a dynamic circular reference after 8 seconds
    circle_publisher_node = Node(
        package='traj_utils',
        executable='circle_publisher',
        name='circle_publisher',
        output='screen',
        parameters=[{
            'radius': 2.0,
            'omega': 1.0,
            'center_x': -18.0,
            'center_y': -1.0,
            'center_z': 2.0,
        }]
    )

    delayed_circle_command = TimerAction(
        period=8.0,
        actions=[circle_publisher_node]
    )

    return LaunchDescription([
        simulator_launch,
        rviz_node,
        delayed_circle_command
    ])
