import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'ocs2_robotic_assets') + '/resources/mobile_manipulator/realman/urdf/rm_75_6f.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'ocs2_mobile_manipulator') + '/config/realman/task.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=get_package_share_directory(
                'ocs2_mobile_manipulator') + '/auto_generated/realman'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory('ocs2_mobile_manipulator_ros') + "/rviz/realman_esdf.rviz"
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mobile_manipulator',
            output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rvizconfig")]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            prefix="",
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                }
            ]
        ),

        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_target',
            name='mobile_manipulator_target',
            prefix="",
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                }
            ]
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
