import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mux_share = get_package_share_directory('twist_mux')
    my_share = get_package_share_directory('my_bot')
    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            # twist_mux 의 출력(/cmd_vel_out)을 dieu_khien 이 구독하는 /cmd_vel 로 연결
            remappings=[('/cmd_vel_out', '/cmd_vel')],
            parameters=[
                os.path.join(mux_share, 'config', 'twist_mux_locks.yaml'),
                os.path.join(my_share, 'config', 'twist_mux.yaml'),
            ],
        ),
    ])
