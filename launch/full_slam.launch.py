from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    full_slam = LaunchConfiguration('full_slam')
    saved_map_file = LaunchConfiguration('saved_map_file')
    point_lio_config = LaunchConfiguration('point_lio_config')
    elevation_config = LaunchConfiguration('elevation_config')
    package_share = FindPackageShare('autonomy_light')
    default_point_lio_config = PathJoinSubstitution([package_share, 'config', 'point_lio_mid360.yaml'])
    default_elevation_config = PathJoinSubstitution([package_share, 'config', 'elevation_map.yaml'])
    full_slam_config = PathJoinSubstitution([package_share, 'config', 'full_slam.yaml'])
    localization_config = PathJoinSubstitution([package_share, 'config', 'localization.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('full_slam', default_value='true'),
        DeclareLaunchArgument('saved_map_file', default_value=''),
        DeclareLaunchArgument('point_lio_config', default_value=default_point_lio_config),
        DeclareLaunchArgument('elevation_config', default_value=default_elevation_config),
        Node(
            package='autonomy_light',
            executable='autonomy_light_pointlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[point_lio_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autonomy_light',
            executable='full_slam_node',
            name='full_slam',
            output='screen',
            parameters=[full_slam_config, {'use_sim_time': use_sim_time}],
            condition=IfCondition(full_slam),
        ),
        Node(
            package='autonomy_light',
            executable='localization_node',
            name='saved_map_localization',
            output='screen',
            parameters=[localization_config, {
                'use_sim_time': use_sim_time,
                'saved_map_file': saved_map_file,
            }],
            condition=UnlessCondition(full_slam),
        ),
        Node(
            package='autonomy_light',
            executable='elevation_map_node',
            name='elevation_map',
            output='screen',
            parameters=[elevation_config, {'use_sim_time': use_sim_time}],
        ),
    ])
