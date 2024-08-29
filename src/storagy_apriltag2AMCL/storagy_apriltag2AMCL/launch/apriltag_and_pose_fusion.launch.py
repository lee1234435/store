from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros2',  # 패키지 이름을 적어주세요
            executable='apriltag_node',   # apriltag_node.py를 실행
            name='apriltag_node',
            output='screen',
            parameters=[
                # 필요하다면 여기에 파라미터를 추가하세요
            ]
        ),
        Node(
            package='apriltag_ros2',  # 패키지 이름을 적어주세요
            executable='pose_fusion_node',  # PoseFusionNode.py를 실행
            name='pose_fusion_node',
            output='screen',
            parameters=[
                # 필요하다면 여기에 파라미터를 추가하세요
            ]
        ),
    ])
