from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_share = FindPackageShare('autonomy_light')
    point_lio_config = PathJoinSubstitution([package_share, 'config', 'point_lio_mid360.yaml'])
    elevation_config = PathJoinSubstitution([package_share, 'config', 'elevation_map.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='autonomy_light',
            executable='autonomy_light_pointlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[point_lio_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autonomy_light',
            executable='elevation_map_node',
            name='elevation_map',
            output='screen',
            parameters=[elevation_config, {'use_sim_time': use_sim_time}],
        ),
    ])
