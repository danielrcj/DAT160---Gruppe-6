import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Namespaces for each robot
    first_tb3 = 'tb3_0'
    second_tb3 = 'tb3_1'

    # Declare use_sim_time argument
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Starting bug_robot nodes with follow_side parameter
    bug_robot_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='bug_robot',
        name='bug_robot_tb3_0',
        output='screen',
        arguments=[first_tb3, 'right'],  # First robot follows the right wall
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bug_robot_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='bug_robot',
        name='bug_robot_tb3_1',
        output='screen',
        arguments=[second_tb3, 'left'],  # Second robot follows the left wall
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        sim_time_arg,
        bug_robot_tb3_0,
        bug_robot_tb3_1,
    ])
