#****************************************************
# This is a ROS2 launch file
#****************************************************

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    pekf_slam_share = FindPackageShare('pekf_slam').find('pekf_slam')
    print('pekf_slam_share: {}'.format(pekf_slam_share))
    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=pekf_slam_share + '/params/param.yaml',
        description='Path to the param file'
    )

    param_substitutions = {
        'use_sim_time': use_sim_time,}
    
    configured_params = RewrittenYaml(
                source_file=params_file,
                root_key=namespace,
                param_rewrites=param_substitutions,
                convert_types=True)
    
    laser_to_pc_node = Node(
        package='pekf_slam',
        executable='scan2pc.py',
        name='laser_to_pc',
        output='screen',
        namespace=namespace,
        parameters=[configured_params],
        remappings=[('scan', 'base_scan')]
    )

    pekf_slam_node = Node(
        package='pekf_slam',
        executable='pekf_slam',
        name='pekf_slam_node',
        namespace=namespace,
        output='screen',
        parameters=[configured_params]
    )

    return LaunchDescription(
        [ 
          declare_namespace_cmd,
          declare_use_sim_time_cmd,
          declare_params_file,
          laser_to_pc_node, 
          pekf_slam_node
        ]
          )
