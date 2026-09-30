import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value=os.path.expanduser('~/aapf_logs'),
        description='Thư mục lưu CSV.')
    rate_arg = DeclareLaunchArgument(
        'rate_hz', default_value='50.0', description='Tần số ghi [Hz].')

    return LaunchDescription([
        output_dir_arg,
        rate_arg,
        Node(
            package='custom_ur_logger',
            executable='human_aapf_logger',
            name='human_aapf_logger',
            output='screen',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'rate_hz': LaunchConfiguration('rate_hz'),
            }],
        ),
    ])
