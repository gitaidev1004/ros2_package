import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Talker 노드 실행
        Node(
            package='launch_example',
            executable='talker',
            name='talker_node',
            output='screen',
        ),
        
        # Listener 노드 실행
        Node(
            package='launch_example',
            executable='listener',
            name='listener_node',
            output='screen',
        ),
        
        # Launch 정보 출력
        LogInfo(
            condition=None,
            msg="Launching Talker and Listener Nodes!"
        )
    ])
