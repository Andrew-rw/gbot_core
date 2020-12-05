import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file = os.path.join(
        get_package_share_directory('gbot_core'),
        'urdf',
        'head_2d.urdf')
    with open(urdf_file, 'r') as infp:
        urdf = infp.read()

    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': urdf
        }]
    )

    cartographer_config_files_dir = os.path.join(
        get_package_share_directory('gbot_core'),
        'configuration_files')

    start_cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            cartographer_config_files_dir,
            '-configuration_basename',
            'gbot_lidar_2d.lua'
        ])

    # Additional node which converts Cartographer map into ROS occupancy grid map. 
    # Not used and can be skipped in this case
    start_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05'])

    ld = LaunchDescription()
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_cartographer_node)
    ld.add_action(start_occupancy_grid_node)

    return ld