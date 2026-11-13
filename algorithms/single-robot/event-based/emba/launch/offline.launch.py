from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    def arg(name): return LaunchConfiguration(name).perform(context)
    share=Path(get_package_share_directory('emba_ros2'))
    params={}
    for key in ('events','calibration','trajectory'):
        path=Path(arg(key)).expanduser().resolve()
        if not path.is_file(): raise FileNotFoundError(f'{key}: {path}')
        params[key]=str(path)
    params['output']=str(Path(arg('output')).expanduser().resolve())
    for key in ('start_ns','stop_ns','seed_step_ns','trajectory_offset_ns'):
        params[key]=int(arg(key))
    for key in ('gx','gy'):
        if arg(key):
            path=Path(arg(key)).expanduser().resolve()
            if not path.is_file(): raise FileNotFoundError(str(path))
            params[key]=str(path)
    nodes=[Node(package='emba_ros2',executable='emba_node',name='emba',namespace='emba',
                parameters=[str(share/'config/serial.yaml'),params],output='screen',
                additional_env={'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','VECLIB_MAXIMUM_THREADS':'1'})]
    if arg('rviz').lower()=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/emba.rviz')]))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k) for k in ('events','calibration','trajectory','output')]+
        [DeclareLaunchArgument('start_ns',default_value='-1'),DeclareLaunchArgument('stop_ns',default_value='-1'),
         DeclareLaunchArgument('seed_step_ns',default_value='0'),DeclareLaunchArgument('trajectory_offset_ns',default_value='0'),
         DeclareLaunchArgument('gx',default_value=''),DeclareLaunchArgument('gy',default_value=''),
         DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
