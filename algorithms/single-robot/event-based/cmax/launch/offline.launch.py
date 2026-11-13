from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    share=Path(get_package_share_directory('cmax_slam_ros2'))
    def arg(n): return LaunchConfiguration(n).perform(context)
    profile=arg('profile')
    if profile not in ('synthetic','motorized_mount'): raise ValueError('unknown profile')
    stream=Path(arg('events')).expanduser().resolve()
    camera=Path(arg('calibration')).expanduser().resolve() if arg('calibration') else Path(str(stream)+'.camera.yaml')
    if not stream.is_file() or not camera.is_file(): raise FileNotFoundError('convert bag first: events and its camera YAML required')
    results=Path(arg('results')).expanduser().resolve(); results.mkdir(parents=True,exist_ok=True)
    if any((results/f).exists() for f in ('trajectory.tum','panorama.png','panorama.raw.yml.gz')):
        raise FileExistsError('use a fresh results directory to preserve previous run')
    nodes=[Node(package='cmax_slam_ros2',executable='cmax_slam_node',name='cmax_slam',namespace='cmax',
        parameters=[str(share/'config'/f'{profile}.yaml'),str(camera),
            {'events_file':str(stream),'trajectory_file':str(results/'trajectory.tum'),'map_file':str(results/'panorama'),
             'use_sim_time':False}],output='screen')]
    if arg('rviz').lower()=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/cmax.rviz')]))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('events'),DeclareLaunchArgument('calibration',default_value=''),
        DeclareLaunchArgument('profile',default_value='synthetic'),DeclareLaunchArgument('results'),
        DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
