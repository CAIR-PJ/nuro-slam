from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('cmax_slam_ros2'))
    return LaunchDescription([Node(package='cmax_slam_ros2',executable='cmax_slam_node',name='cmax_slam',namespace='cmax',
        parameters=[str(share/'config/synthetic.yaml')],output='screen')])
