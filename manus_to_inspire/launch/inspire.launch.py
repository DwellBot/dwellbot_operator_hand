from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manus_ros2',
            executable='manus_ros2',
            name='manus_ros2',
            output='screen'),

    ])


"""         Node(
            package='manus_to_inspire',
            executable='ik',
            name='manus_ik_to_ros2',
            output='screen',
            emulate_tty=True,
            parameters=[
            {"isLeft": False},
            ]
        ) """