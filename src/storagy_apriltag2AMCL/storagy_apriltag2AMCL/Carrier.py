


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from math import radians, sin, cos
import time

class TagParkingController(Node):
    def __init__(self):
        super().__init__('tag_parking_controller')

        self.yaw_goal = radians(-90)  # 목표 yaw 값 (-90도)
        self.yaw_goal_tolerance = radians(3.0)  # 허용 오차 (±3도)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Pose 업데이트를 위한 publisher
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.send_nav_goal()

    def send_nav_goal(self):
        """
        Nav2_Goal
        """

        coordinates = (2.2041015625, -1.54281485080719, 0.00250244140625)
        goal_msg = self.create_navigation_goal(coordinates)

        self.nav_to_pose_client.wait_for_server()
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_goal_response_callback)

    def create_navigation_goal(self, coordinates):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = coordinates[0]
        goal_msg.pose.pose.position.y = coordinates[1]
        goal_msg.pose.pose.position.z = coordinates[2]

        q = self.quaternion_from_euler(0, 0, self.yaw_goal)
        goal_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return goal_msg

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Nav2 goal was rejected.')
            return
        
        self.get_logger().info('Nav2 goal accepted, waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.get_logger().info('Nav2 goal reached! Waiting for 2 seconds before updating localization...')
        
        # 2초 대기
        time.sleep(2)
        
        # 도착한 위치와 yaw로 localization 업데이트
        self.update_localization()
        
        # 노드 종료
        self.get_logger().info('Localization updated. Shutting down the node.')
        self.destroy_node()

    def update_localization(self):
        """
        도착지점으로 Localization update
        """
        # 목표 지점에서 로봇의 위치를 설정하여 localization 업데이트
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.pose.position.x = 2.1041015625
        pose_msg.pose.pose.position.y = -1.44281485080719
        pose_msg.pose.pose.position.z = 0.00250244140625

        q = self.quaternion_from_euler(0, 0, self.yaw_goal)
        pose_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        
        pose_msg.pose.covariance = [0.0] * 36

        # Pose를 퍼블리시하여 localization 업데이트
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info('Published updated localization pose.')

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = TagParkingController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
