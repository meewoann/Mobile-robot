from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('sim_description')

    # ---------- URDF ----------
    xacro_file = os.path.join(share_dir, 'urdf', 'sim.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # ---------- RViz ----------
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # ---------- Args ----------
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/meewoan/Comvi_ws/src/arena_map.yaml'
    )

    show_gui = LaunchConfiguration('gui')
    map_yaml = LaunchConfiguration('map')

    # ---------- Nodes ----------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',   
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': False,
                'topic_name': 'map',
                'frame_id': 'map'
            }],
            output='screen'
        )


    static_tf_map_to_base = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                    '--yaw', '0', '--pitch', '0', '--roll', '0', 
                    '--frame-id', 'map', 
                    '--child-frame-id', 'base_link']
        )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','map','odom']
    )

    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','odom','base_link']
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )


    return LaunchDescription([
        gui_arg,
        map_arg,

        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,

        static_map_to_odom,
        static_odom_to_base,

        map_server_node,
        lifecycle_manager_map,  

        static_tf_map_to_base,
        rviz_node
    ])
