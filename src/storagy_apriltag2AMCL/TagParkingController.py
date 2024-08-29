

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
from math import radians, sin, cos
import time

class TagParkingController(Node):
    def __init__(self):
        super().__init__('tag_parking_controller')
        
        self.current_yaw = None
        self.yaw_timeout = 5.0
        self.current_step = 0

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',  # 실제 사용 중인 IMU 토픽 이름으로 변경
            self.imu_callback,
            10)


        self.start_time = self.get_clock().now()
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yaw_goal_tolerance = radians(3.0)  # 목표 yaw에 대한 허용 오차 (5도)

        # 서비스 서버를 생성하여 동작을 시작하도록 설정
        self.srv = self.create_service(Empty, '/start_parking', self.start_parking_callback)

    def start_parking_callback(self, request, response):
        self.get_logger().info('Start parking service called. Beginning parking sequence...')
        self.start_initial_nav_goal()
        return response

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        self.current_yaw = self.get_yaw_from_quaternion(orientation_q)
        self.get_logger().info(f'Received IMU data. Current yaw: {self.current_yaw:.2f} radians')

    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = tf2_ros.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        return yaw

    def start_initial_nav_goal(self):
        """
        주차 태그 위치를 고려한 첫번째 위치 주행
        """
        self.handle_initial_navigation()

    def handle_initial_navigation(self):
        """
        IMU 토픽 구독 후 해당 정보 활용
        """
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self.current_yaw is None:
            if elapsed_time < self.yaw_timeout:
                self.get_logger().warn('Waiting for valid IMU data...')
                return
            else:
                self.get_logger().warn(f'IMU data not received in {self.yaw_timeout} seconds. Using default yaw.')
                self.current_yaw = radians(100)

        coordinates = (3.41477952003479, 0.5997087359428406, 0.0)
        
        goal_msg = self.create_navigation_goal(coordinates)

        self.nav_to_pose_client.wait_for_server()
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_goal_response_callback)





    def create_navigation_goal(self, coordinates):
        """
        Nav2_Goal
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = coordinates[0]
        goal_msg.pose.pose.position.y = coordinates[1]
        goal_msg.pose.pose.position.z = coordinates[2]
        
        q = self.quaternion_from_euler(0, 0, self.current_yaw)
        goal_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return goal_msg

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Nav2 goal was rejected.')
            return
        time.sleep(2)  # 2초 대기

        # 로봇을 정지시키기 위해 cmd_vel에 속도를 0으로 설정
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        if self.check_yaw_within_tolerance():
            #self.update_robot_position()
            self.get_logger().info('Robot position and yaw are within tolerance. Localization updated.')
            
        else:
            self.get_logger().warn("Yaw is outside the tolerance. Attempting to correct yaw.")
            self.correct_yaw()
        
        
        self.get_logger().info('Nav2 goal accepted, waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.nav_result_callback)
    

    def correct_yaw(self):
        """
        Goal의 target Yaw값과 IMU를 통한 현재 Yaw 비교 보정
        """
        target_yaw = radians(90)
        yaw_error = self.current_yaw - target_yaw

        twist_msg = Twist()
        twist_msg.angular.z = -0.1 if yaw_error > 0 else 0.1
        self.cmd_vel_pub.publish(twist_msg)

        time.sleep(1)  # 회전 보정을 위한 시간

        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        self.get_logger().info('Yaw correction applied. Retrying localization update...')
        #self.update_robot_position()
        
        
    def check_yaw_within_tolerance(self):
        """
        허용범위 내 판단
        """
        # 현재 yaw 값이 목표 yaw 값 범위 내에 있는지 확인
        if self.current_yaw is None:
            return False

        goal_yaw = radians(90)  # 목표 yaw 값
        yaw_error = abs(self.current_yaw - goal_yaw)
        
        return yaw_error <= self.yaw_goal_tolerance

    def update_robot_position(self):
        """
        Localization update
        """
        # 목표 지점에서 로봇의 위치를 설정
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        
        pose_msg.pose.pose.position.x = 3.41477952003479
        pose_msg.pose.pose.position.y = 0.5674043893814087
        pose_msg.pose.pose.position.z = 0.0



    def nav_result_callback(self, future):
        self.get_logger().info('Nav2 goal reached! Waiting for 2 seconds before starting forward movement...')
        #time.sleep(2)
        self.move_forward_and_rotate()

    def move_forward_and_rotate(self):
        """
        추가 주행
        """
        time.sleep(14)
        self.execute_movement(0.3, 2.5)  # 직진
        time.sleep(10)
        self.execute_rotation(-0.5, 1.5)  # 회전
        self.current_step += 1  # 다음 단계로 이동
        self.get_logger().info('Movement and rotation completed.')
        self.enter_idle_state()  # 대기 상태로 들어감

    def execute_movement(self, linear_velocity, duration):
        """
        타이머를 활용한 직진 동작
        """
        self.get_logger().info('Starting forward movement.')

        start_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time < duration:
                self.get_logger().info(f'Moving forward... Elapsed time: {elapsed_time:.2f} seconds')
                twist_msg = Twist()
                twist_msg.linear.x = linear_velocity
                self.cmd_vel_pub.publish(twist_msg)
            else:
                break

            time.sleep(0.1)

        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('Stopped after moving forward.')

    def execute_rotation(self, angular_velocity, duration):
        """
        타이머를 활용한 회전동작
        """
        self.get_logger().info('Starting rotation.')

        start_time = time.time()
        duration_ro = 7

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time < duration_ro:
                self.get_logger().info(f'Rotating... Elapsed time: {elapsed_time:.2f} seconds')
                twist_msg = Twist()
                twist_msg.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist_msg)
            else:
                break

            time.sleep(0.1)

        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('Stopped after rotation.')

    def enter_idle_state(self):
        self.get_logger().info('Entering idle state. No further actions will be taken.')
        # 노드가 대기 상태로 유지되도록 함
        rclpy.spin_once(self, timeout_sec=1.0)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Roll, Pitch, Yaw  변환
        """
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




