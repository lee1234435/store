import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='storagy_apriltag_tracking',
            executable='aruco_node',
            name='aruco_node',
            output='screen'
        ),
        Node(
            package='storagy_apriltag_tracking',
            executable='cmd_node',
            name='aruco_yolo_cmd_node',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
