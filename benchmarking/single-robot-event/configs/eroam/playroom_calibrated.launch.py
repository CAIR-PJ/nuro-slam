from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('events', default_value=''),
        DeclareLaunchArgument('results', default_value='/tmp/eroam_ros2_results'),
        DeclareLaunchArgument('packet_size', default_value='500'),
        Node(package='eroam_ros2', executable='eroam_node', output='screen', parameters=[{
            'events_file': LaunchConfiguration('events'),
            'results_dir': LaunchConfiguration('results'),
            'events_per_packet': LaunchConfiguration('packet_size'),
            'dvs.width': 128, 'dvs.height': 128,
            'dvs.intrinsic_matrix': [91.4014729896821, 0.0, 64.0, 0.0, 91.4014729896821, 64.0, 0.0, 0.0, 1.0],
            'dvs.dist_params': [0.0,0.0,0.0,0.0,0.0],
            'options.min_frame_evs': 10,
            'output.pose_result_file': [LaunchConfiguration('results'),'/trajectory.tum'],
            'output.panoramic_output_file': [LaunchConfiguration('results'),'/panorama.png'],
        }])
    ])
