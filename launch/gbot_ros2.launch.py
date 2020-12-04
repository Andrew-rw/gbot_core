import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Include the default rplidar_ros launch file with default parameters.
    # If you need to customize any rplidar parameters uncomment and use the
    # the launch config below.
    rplidar_launchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar.launch.py')),
        launch_arguments={}.items()
    )

    # This launch action (node) was copied from rplidar.launch.py
    # rplidar_launchDescription = Node(
    #     name='rplidar_node',
    #     package='rplidar_ros',
    #     executable='rplidar_composition',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': '/dev/ttyUSB0',
    #         'serial_baudrate': 115200,  # A1 / A2
    #         # 'serial_baudrate': 256000, # A3
    #         'frame_id': 'laser',
    #         'inverted': False,
    #         'angle_compensate': True
    #     }],
    # )

    gbot_ros2_no_lidar_launchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gbot_core'),
                'launch',
                'gbot_ros2_no_lidar.launch.py')),
        launch_arguments={}.items()
    )

    ld = LaunchDescription()
    ld.add_action(rplidar_launchDescription)
    ld.add_action(gbot_ros2_no_lidar_launchDescription)

    return ld

