import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_display = os.path.join(
      get_package_share_directory('gbot_core'),
      'rviz',
      'demo_ros2.rviz')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='gbot_core_rviz2_node',
        output='screen',
        arguments=['-d', rviz_display])

    return LaunchDescription([rviz2_node])