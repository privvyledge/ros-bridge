import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
                name='input_msg_is_stamped',
                default_value='False'
        ),
        launch_ros.actions.Node(
            package='carla_twist_to_control',
            executable='carla_twist_to_control',
            name=launch.substitutions.LaunchConfiguration('role_name'),
            output='screen',
            emulate_tty='True',
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                    'input_msg_is_stamped': launch.substitutions.LaunchConfiguration('input_msg_is_stamped')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
