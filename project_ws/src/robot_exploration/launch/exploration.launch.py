#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    explore_rate = LaunchConfiguration('explore_rate')
    min_frontier_distance = LaunchConfiguration('min_frontier_distance')
    max_goal_distance = LaunchConfiguration('max_goal_distance')
    robot_radius = LaunchConfiguration('robot_radius')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_explore_rate_cmd = DeclareLaunchArgument(
        'explore_rate',
        default_value='1.0',
        description='Rate of exploration planning (Hz)'
    )
    
    declare_min_frontier_distance_cmd = DeclareLaunchArgument(
        'min_frontier_distance',
        default_value='0.8',
        description='Minimum distance between frontier goals (meters)'
    )
    
    declare_max_goal_distance_cmd = DeclareLaunchArgument(
        'max_goal_distance',
        default_value='8.0',
        description='Maximum distance for exploration goals (meters)'
    )
    
    declare_robot_radius_cmd = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.35',
        description='Robot radius for obstacle avoidance (meters)'
    )

    # Autonomous exploration node
    exploration_node = Node(
        package='robot_exploration',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'explore_rate': explore_rate},
            {'min_frontier_distance': min_frontier_distance},
            {'max_goal_distance': max_goal_distance},
            {'robot_radius': robot_radius}
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/map', '/map'),
            ('/navigate_to_pose', '/navigate_to_pose')
        ]
    )
    
    # Delay exploration start to ensure navigation is ready
    delayed_exploration = TimerAction(
        period=5.0,  # Wait 5 seconds before starting exploration
        actions=[exploration_node]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_explore_rate_cmd)
    ld.add_action(declare_min_frontier_distance_cmd)
    ld.add_action(declare_max_goal_distance_cmd)
    ld.add_action(declare_robot_radius_cmd)
    
    # Add delayed exploration node
    ld.add_action(delayed_exploration)
    
    return ld