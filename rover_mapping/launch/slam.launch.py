#!/usr/bin/env python3
"""
slam.launch.py — SLAM mapping launch for UGV Rover

Assumes rover_bringup robot.launch.py is already running, which provides:
  - LD19 LIDAR              → /scan
  - odom_publisher          → /odom + odom→base_footprint TF
  - robot_state_publisher   → base_footprint→base_link TF (from URDF)

This launch file adds only:
  - SLAM Toolbox (async, lifecycle-managed)

Full mapping session workflow:
  # Terminal 1 — robot hardware
  ros2 launch rover_bringup robot.launch.py

  # Terminal 2 — SLAM
  ros2 launch rover_mapping slam.launch.py

  # Terminal 3 — drive to build the map
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # Terminal 4 — save map when done
  ros2 launch rover_mapping map_saver.launch.py map_name:=my_room

Prerequisites:
  sudo apt install ros-jazzy-slam-toolbox
  sudo apt install ros-jazzy-nav2-map-server
"""

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
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


def generate_launch_description():

    # ── Arguments ───────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── SLAM Toolbox ─────────────────────────────────────────────────────────
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
        slam_delayed,
    ])