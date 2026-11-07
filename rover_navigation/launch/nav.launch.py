import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_rover_mapping = get_package_share_directory('rover_mapping')
    pkg_rover_nav = get_package_share_directory('rover_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_rover_mapping, 'maps', 'den', 'map.yaml'),
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_rover_nav, 'config', 'rover_nav2.yaml'),
    )

    nav2 = IncludeLaunchDescription(
        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py'),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([map_arg, params_arg, nav2])