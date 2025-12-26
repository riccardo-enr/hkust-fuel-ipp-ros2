import os
import yaml
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Helper to load yaml
    def load_yaml(path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    # Arguments
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_z = LaunchConfiguration('init_z')
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    odometry_topic = LaunchConfiguration('odometry_topic')
    controller_type = LaunchConfiguration('controller_type')

    # Paths and Configs
    so3_control_share = get_package_share_directory('so3_control')
    local_sensing_share = get_package_share_directory('local_sensing_node')
    plan_manage_share = get_package_share_directory('plan_manage')

    gains_config = load_yaml(os.path.join(so3_control_share, 'config', 'gains_hummingbird.yaml'))
    corrections_config = load_yaml(os.path.join(so3_control_share, 'config', 'corrections_hummingbird.yaml'))
    camera_config = load_yaml(os.path.join(local_sensing_share, 'params', 'camera.yaml'))
    fast_planner_config = load_yaml(os.path.join(plan_manage_share, 'config', 'fast_planner.yaml'))['fast_planner_node']['ros__parameters']
    
    plan_bringup_share = get_package_share_directory('plan_bringup')
    # Load both MPPI configs (acceleration and thrust+quaternion variants)
    mppi_acc_config = load_yaml(os.path.join(plan_bringup_share, 'config', 'mppi_acc.yaml'))['mppi_control']['ros__parameters']
    mppi_tq_config = load_yaml(os.path.join(plan_bringup_share, 'config', 'mppi_tq.yaml'))['mppi_control']['ros__parameters']

    # Controller remappings logic
    # If MPPI: so3_control node is bypassed (but simulator still needs so3_cmd)
    # We will launch mppi_control and it will publish directly to so3_cmd

    return LaunchDescription([
        DeclareLaunchArgument('init_x', default_value='-19.99'),
        DeclareLaunchArgument('init_y', default_value='-1.0'),
        DeclareLaunchArgument('init_z', default_value='1.0'),
        DeclareLaunchArgument('map_size_x', default_value='40.0'),
        DeclareLaunchArgument('map_size_y', default_value='20.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),
        DeclareLaunchArgument('odometry_topic', default_value='/state_ukf/odom'),
        DeclareLaunchArgument('map_type', default_value='random_forest', description='Map generator type: random_forest, empty_world, office, office2, office3, or pillar'),
        DeclareLaunchArgument('controller_type', default_value='so3', description='Controller type: so3, mppi_acc, or mppi_tq'),
        
        # Map Generator (PCD)
        Node(
            package='map_generator',
            executable='map_pub',
            name='map_pub',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_type'), "' in ['office', 'office2', 'office3', 'pillar']"])),
            output='screen',
            arguments=[PathJoinSubstitution([
                FindPackageShare('map_generator'),
                'resource',
                PythonExpression(["'", LaunchConfiguration('map_type'), "' + '.pcd'"])
            ])],
        ),

        # Map Generator (Random Forest)
        Node(
            package='map_generator',
            executable='random_forest',
            name='random_forest',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_type'), "' == 'random_forest'"])),
            output='screen',
            parameters=[{
                'init_state_x': init_x,
                'init_state_y': init_y,
                'map/x_size': map_size_x,
                'map/y_size': map_size_y,
                'map/z_size': map_size_z,
                'map/resolution': 0.1,
                'ObstacleShape/seed': 1,
                'map/obs_num': 80,
                'map/circle_num': 80,
                'ObstacleShape/lower_rad': 0.5,
                'ObstacleShape/upper_rad': 0.8,
                'ObstacleShape/lower_hei': 0.0,
                'ObstacleShape/upper_hei': 3.0,
                'ObstacleShape/radius_l': 0.7,
                'ObstacleShape/radius_h': 0.8,
                'ObstacleShape/z_l': 0.7,
                'ObstacleShape/z_h': 0.8,
                'ObstacleShape/theta': 0.5,
                'sensing/radius': 5.0,
                'sensing/rate': 10.0,
            }],
            remappings=[
                ('odometry', odometry_topic),
            ]
        ),
        
        # Map Generator (Empty World)
        Node(
            package='map_generator',
            executable='empty_world',
            name='empty_world',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('map_type'), "' == 'empty_world'"])),
            output='screen',
            parameters=[{
                'map/x_size': map_size_x,
                'map/y_size': map_size_y,
                'map/resolution': 0.1,
                'sensing/rate': 10.0,
            }],
            remappings=[
                ('odometry', odometry_topic),
            ]
        ),
        # Traj Utils Process Msg
        Node(
            package='traj_utils',
            executable='process_msg',
            name='process_msg',
            output='screen'
        ),

        # Quadrotor Simulator
        Node(
            package='so3_quadrotor_simulator',
            executable='quadrotor_simulator_so3',
            name='quadrotor_simulator_so3',
            output='screen',
            parameters=[{
                'rate/odom': 200.0,
                'simulator/init_state_x': init_x,
                'simulator/init_state_y': init_y,
                'simulator/init_state_z': init_z,
            }],
            remappings=[
                ('odom', '/visual_slam/odom'),
                ('cmd', 'so3_cmd'),
                ('force_disturbance', 'force_disturbance'),
                ('moment_disturbance', 'moment_disturbance'),
            ]
        ),

        # MPPI Acceleration Control (Composable Node)
        ComposableNodeContainer(
            name='mppi_control_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(PythonExpression(["'", controller_type, "' == 'mppi_acc'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='mppi_control',
                    plugin='mppi_control::MPPIAccNode',
                    name='mppi_control',
                    parameters=[
                        fast_planner_config,
                        gains_config,
                        mppi_acc_config
                    ],
                    remappings=[
                        ('odom', '/state_ukf/odom'),
                        ('planning/pos_cmd', '/planning/pos_cmd'),
                        ('so3_cmd', 'so3_cmd'),
                        ('/sdf_map/occupancy_all', '/mppi_control/sdf_map/occupancy_all'),
                        ('/sdf_map/occupancy_local', '/mppi_control/sdf_map/occupancy_local'),
                        ('/sdf_map/free_all', '/mppi_control/sdf_map/free_all'),
                        ('/sdf_map/free_local', '/mppi_control/sdf_map/free_local'),
                        ('/sdf_map/occupancy_local_inflate', '/mppi_control/sdf_map/occupancy_local_inflate'),
                        ('/sdf_map/unknown', '/mppi_control/sdf_map/unknown'),
                        ('/sdf_map/esdf', '/mppi_control/sdf_map/esdf'),
                        ('/sdf_map/update_range', '/mppi_control/sdf_map/update_range'),
                        ('/sdf_map/depth_cloud', '/mppi_control/sdf_map/depth_cloud'),
                        ('/sdf_map/explored_volume', '/mppi_control/sdf_map/explored_volume'),
                    ]
                )
            ],
            output='screen',
        ),

        # MPPI Thrust+Quaternion Control (Composable Node)
        ComposableNodeContainer(
            name='mppi_control_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(PythonExpression(["'", controller_type, "' == 'mppi_tq'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='mppi_control',
                    plugin='mppi_control::MPPITqNode',
                    name='mppi_control',
                    parameters=[
                        fast_planner_config,
                        gains_config,
                        mppi_tq_config
                    ],
                    remappings=[
                        ('odom', '/state_ukf/odom'),
                        ('planning/pos_cmd', '/planning/pos_cmd'),
                        ('so3_cmd', 'so3_cmd'),
                        ('/sdf_map/occupancy_all', '/mppi_control/sdf_map/occupancy_all'),
                        ('/sdf_map/occupancy_local', '/mppi_control/sdf_map/occupancy_local'),
                        ('/sdf_map/free_all', '/mppi_control/sdf_map/free_all'),
                        ('/sdf_map/free_local', '/mppi_control/sdf_map/free_local'),
                        ('/sdf_map/occupancy_local_inflate', '/mppi_control/sdf_map/occupancy_local_inflate'),
                        ('/sdf_map/unknown', '/mppi_control/sdf_map/unknown'),
                        ('/sdf_map/esdf', '/mppi_control/sdf_map/esdf'),
                        ('/sdf_map/update_range', '/mppi_control/sdf_map/update_range'),
                        ('/sdf_map/depth_cloud', '/mppi_control/sdf_map/depth_cloud'),
                        ('/sdf_map/explored_volume', '/mppi_control/sdf_map/explored_volume'),
                    ]
                )
            ],
            output='screen',
        ),

        # SO3 Control (Component)
        ComposableNodeContainer(
            name='so3_control_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(PythonExpression(["'", controller_type, "' == 'so3'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='so3_control',
                    plugin='so3_control::SO3ControlComponent',
                    name='so3_control',
                    parameters=[
                        gains_config,
                        corrections_config,
                        {
                            'mass': 0.98,
                            'use_angle_corrections': False,
                            'use_external_yaw': False,
                            'gains/rot/z': 1.0,
                            'gains/ang/z': 0.1,
                        }
                    ],
                    remappings=[
                        ('odom', '/state_ukf/odom'),
                        ('position_cmd', '/planning/pos_cmd'),
                        ('motors', 'motors'),
                        ('corrections', 'corrections'),
                        ('so3_cmd', 'so3_cmd'),
                    ]
                )
            ],
            output='screen',
        ),

        # Disturbance Generator
        Node(
            package='so3_disturbance_generator',
            executable='so3_disturbance_generator',
            name='so3_disturbance_generator',
            output='screen',
            remappings=[
                ('odom', '/visual_slam/odom'),
                ('noisy_odom', '/state_ukf/odom'),
                ('correction', '/visual_slam/correction'),
                ('force_disturbance', 'force_disturbance'),
                ('moment_disturbance', 'moment_disturbance'),
            ]
        ),

        # Odom Visualization
        Node(
            package='odom_visualization',
            executable='odom_visualization',
            name='odom_visualization',
            namespace='odom_visualization',
            output='screen',
            parameters=[{
                'color/a': 1.0,
                'color/r': 0.0,
                'color/g': 0.0,
                'color/b': 1.0,
                'cmd_color/a': 0.5,
                'cmd_color/r': 1.0,
                'cmd_color/g': 0.0,
                'cmd_color/b': 0.0,
                'covariance_scale': 100.0,
                'robot_scale': 1.0,
            }],
            remappings=[
                ('odom', '/visual_slam/odom'),
                ('cmd', '/planning/pos_cmd'),
            ]
        ),

        # Local Sensing (PCL Render Node)
        Node(
            package='local_sensing_node',
            executable='pcl_render_node',
            name='pcl_render_node',
            output='screen',
            parameters=[
                camera_config,
                {
                    'sensing_horizon': 5.0,
                    'sensing_rate': 30.0,
                    'estimation_rate': 30.0,
                    'map/x_size': map_size_x,
                    'map/y_size': map_size_y,
                    'map/z_size': map_size_z,
                }
            ],
            remappings=[
                ('global_map', '/map_generator/global_cloud'),
                ('odometry', odometry_topic),
            ]
        )
    ])
