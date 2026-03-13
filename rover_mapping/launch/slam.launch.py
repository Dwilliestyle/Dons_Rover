#!/usr/bin/env python3
"""
slam.launch.py — SLAM mapping launch for UGV Rover

Assumes rover_bringup robot.launch.py is already running, which provides:
  - LD19 LIDAR              -> /scan
  - odom_publisher          -> /odom + odom->base_footprint TF
  - robot_state_publisher   -> base_footprint->base_link TF (from URDF)

This launch file adds:
  - Camera viewer (rover_vision)
  - SLAM Toolbox (async, lifecycle-managed)

Full mapping session workflow:
  # Terminal 1 - robot hardware
  ros2 launch rover_bringup robot.launch.py

  # Terminal 2 - SLAM + camera
  ros2 launch rover_mapping slam.launch.py

  # Terminal 3 - drive to build the map
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # Terminal 4 - save map when done
  ros2 launch rover_mapping map_saver.launch.py map_name:=my_room

Optional args:
  ros2 launch rover_mapping slam.launch.py show_window:=false  # SSH without X11

Prerequisites:
  sudo apt install ros-jazzy-slam-toolbox
  sudo apt install ros-jazzy-nav2-map-server
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


def generate_launch_description():

    # -- Arguments -------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Camera image topic to subscribe to'
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Show OpenCV window (set to false when SSH without X11)'
    )
    use_config_arg = DeclareLaunchArgument(
        'use_config',
        default_value='true',
        description='Use config file or command line parameters'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # -- Camera Viewer ---------------------------------------------------------
    rover_vision_config = os.path.join(
        get_package_share_directory('rover_vision'),
        'config',
        'camera_params.yaml'
    )

    camera_viewer = Node(
        package='rover_vision',
        executable='camera_viewer',
        name='camera_viewer',
        parameters=[rover_vision_config],
        output='screen',
    )

    # -- SLAM Toolbox ----------------------------------------------------------
    # Delayed 3 seconds to ensure /scan and /odom are publishing before SLAM
    # attempts its first transform lookup.
    # If you see "waiting for transform" on first run, increase period to 5.0.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_mapping'),
                'config',
                'slam_toolbox_params.yaml'
            ]),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[configure_event],
        )
    )

    slam_delayed = TimerAction(
        period=3.0,
        actions=[slam_node, configure_on_start, activate_event],
    )

    return LaunchDescription([
        use_sim_time_arg,
        image_topic_arg,
        show_window_arg,
        use_config_arg,
        camera_viewer,
        slam_delayed,
    ])