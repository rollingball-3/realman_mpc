import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'ocs2_robotic_assets') + '/resources/mobile_manipulator/realhex/urdf/realhex.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'ocs2_mobile_manipulator') + '/config/realhex/task.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=get_package_share_directory(
                'ocs2_mobile_manipulator') + '/auto_generated/realhex'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_mobile_manipulator_ros'), 'launch/include/mobile_manipulator.launch.py')
            ),
            launch_arguments={
                'rviz': launch.substitutions.LaunchConfiguration('rviz'),
                'debug': launch.substitutions.LaunchConfiguration('debug'),
                'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
                'taskFile': launch.substitutions.LaunchConfiguration('taskFile'),
                'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description() 