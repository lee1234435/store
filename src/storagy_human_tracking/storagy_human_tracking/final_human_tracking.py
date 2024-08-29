import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import time
from std_msgs.msg import Bool
from pyzbar import pyzbar

class final_human_tracking(Node):
    def __init__(self):
        super().__init__('final_human_tracking')
        
        # 카메라에서 RGB 이미지와 Depth 이미지를 수신하기 위한 구독자 설정
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # 로봇의 위치와 속도를 퍼블리시하기 위한 퍼블리셔 설정
        self.pose_publisher = self.create_publisher(PoseStamped, '/person_position', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 기능 활성화 신호를 수신하기 위한 구독자 설정
        self.activation_subscriber = self.create_subscription(
            Bool,
            '/activation_signal',
            self.activation_callback,
            10
        )

        # QR 코드 인식을 위한 퍼블리셔 설정
        self.qr_code_publisher = self.create_publisher(Bool, '/activation_signal', 10)

        # 유틸리티 설정
        self.bridge = CvBridge()
        self.model = YOLO('src/storagy_human_tracking/models/yolov8n.pt')  # YOLO 모델 로드
        self.depth_image = None
        
        # 처음으로 감지된 사람을 추적하기 위한 변수들
        self.person_detected = False
        self.person_position = None
        self.last_known_position = None
        self.last_detection_time = time.time()
        
        # 카메라 파라미터 설정
        self.image_width = 640
        self.image_height = 480
        self.fov_x = 90.0  # X 방향 FOV (도 단위)
        self.fov_y = 60.0  # Y 방향 FOV (도 단위)
        self.pixel_to_meter_x = 2 * np.tan(np.radians(self.fov_x / 2)) / self.image_width
        self.pixel_to_meter_y = 2 * np.tan(np.radians(self.fov_y / 2)) / self.image_height

        # 목표 거리 설정 (로봇과 사람 사이의 거리)
        self.target_distance = 0.4
        
        # 속도 설정
        self.max_linear_speed = 0.175
        self.max_angular_speed = 0.175
        
        # 타이머 설정 (주기적으로 콜백 함수 실행)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 최신 RGB 이미지를 저장하기 위한 변수
        self.latest_rgb_image = None

        # 사람이 감지되지 않을 때 다시 감지 시도하기까지의 대기 시간
        self.detection_timeout = 3.0  # 초 단위

        # 사람이 감지되지 않은 상태에서의 회전 시작 시간
        self.rotation_start_time = None

        # 기능 활성화 상태를 추적하기 위한 변수
        self.is_active = False
        
    def depth_callback(self, msg):
        """
        ROS 이미지 메시지를 OpenCV 이미지로 변환 (Depth 이미지)
        """
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        """
        최신 RGB 이미지를 저장
        """
        self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # QR 코드 인식 및 활성화 신호 퍼블리시
        self.detect_qr_code_and_publish(self.latest_rgb_image)
    
    def activation_callback(self, msg):
        """
        기능 활성화/비활성화 신호를 처리하는 콜백 함수
        """
        self.is_active = msg.data
        if self.is_active:
            self.get_logger().info("Person tracking activated.")
        else:
            self.get_logger().info("Person tracking deactivated.")
            self.reset_tracking()

    def is_same_person(self, current_position, last_position, threshold=0.5):
        """
        두 좌표가 동일한 사람인지 비교하는 함수
        """
        current_x, current_y, current_z = current_position
        last_x, last_y, last_z = last_position
        
        # 두 좌표 간의 유클리드 거리 계산
        distance = np.sqrt((current_x - last_x)**2 + (current_y - last_y)**2 + (current_z - last_z)**2)
        
        # 거리가 threshold 이하라면 동일한 사람으로 간주
        return distance <= threshold

    def reset_tracking(self):
        """
        사람 추적 상태와 위치를 초기화하는 함수
        """
        self.person_detected = False
        self.person_position = None
        self.last_known_position = None
        self.rotation_start_time = None  # 회전 시작 시간 초기화

    def detect_qr_code_and_publish(self, image):
        """
        QR 코드를 인식하고, 특정 QR 코드가 감지되면 활성화 신호를 퍼블리시
        """
        decoded_objects = pyzbar.decode(image)
        for obj in decoded_objects:
            qr_data = obj.data.decode("utf-8")
            self.get_logger().info(f"QR code detected: {qr_data}")
            
            # 특정 QR 코드 데이터에 따라 활성화 또는 비활성화
            if qr_data == "START_TRACKING":
                self.qr_code_publisher.publish(Bool(data=True))
            elif qr_data == "STOP_TRACKING":
                self.qr_code_publisher.publish(Bool(data=False))

    def timer_callback(self):
        """
        YOLO를 사용하여 RGB 이미지에서 객체 감지 수행
        """
        # 활성화 상태가 아니면 추적 기능을 수행하지 않음
        if not self.is_active:
            self.get_logger().info(f"is active : {self.is_active}")
            self.get_logger().info("Person tracking is inactive.")
            return

        # 현재 시간을 저장
        current_time = time.time()
        
        # RGB 또는 Depth 이미지가 아직 수신되지 않았을 때 대기
        if self.latest_rgb_image is None or self.depth_image is None:
            self.get_logger().info("Waiting for images...")
            return
        
        rgb_image = self.latest_rgb_image
        
        # YOLO를 사용하여 RGB 이미지에서 객체 감지 수행
        results = self.model.predict(source=rgb_image)
        
        detected_persons = [] # 사람을 인식했다면 인식된 사람의 x,y, 거리 데이터 저장

        for result in results:
            boxes = result.boxes
            for box in boxes:
                xyxy = box.xyxy[0].int().tolist()  # 좌표를 정수 리스트로 변환
                
                if int(box.cls[0]) == 0:  # 'person' 클래스 인덱스가 0이라고 가정
                    # 바운딩 박스의 중심 계산
                    x1, y1, x2, y2 = xyxy
                    x_center = int((x1 + x2) / 2)
                    y_center = int((y1 + y2) / 2)
                    
                    # 좌표가 Depth 이미지의 범위를 벗어나지 않도록 확인
                    if x_center < 0 or x_center >= self.image_width or y_center < 0 or y_center >= self.image_height:
                        self.get_logger().warn(f"Detected person at out-of-bounds coordinates: ({x_center}, {y_center})")
                        continue
                    
                    try:
                        # 감지된 사람에 대한 Depth 정보 가져오기
                        distance = self.depth_image[y_center, x_center] / 1000.0  # 밀리미터를 미터로 변환
                        
                        # Depth 정보가 유효한지 확인
                        if np.isnan(distance) or distance <= 0:
                            self.get_logger().info(f"Invalid depth value at ({x_center}, {y_center})")
                            continue
                        
                        # 픽셀 좌표를 미터 단위로 변환 (FOV 변환 적용)
                        x_center_m = (x_center - self.image_width / 2) * distance * self.pixel_to_meter_x
                        y_center_m = (y_center - self.image_height / 2) * distance * self.pixel_to_meter_y
                        
                        # 디버깅을 위한 좌표와 거리 로그 출력
                        self.get_logger().info(f"Converted coordinates: x={x_center_m:.2f}, y={y_center_m:.2f}, z={distance:.2f}")
                        
                        # 감지된 사람을 저장
                        detected_persons.append((x_center_m, y_center_m, distance))
                    
                    except IndexError as e:
                        self.get_logger().error(f"IndexError encountered: {e}")
                        continue
        
        if detected_persons:
            # 사람을 인식했다면
            valid_persons = []
            
            for p in detected_persons:
                distance = p[2]
                
                # 거리 값의 유효성 검증
                if np.isnan(distance) or distance <= 0.1:
                    self.get_logger().warn(f"Invalid distance value: {distance:.2f}m, ignoring this detection.")
                    continue  # 유효하지 않은 거리 값은 무시
                
                valid_persons.append(p)
            
            if valid_persons:
                # 감지된 사람들 중에서 가장 가까운 사람 선택
                valid_persons.sort(key=lambda p: p[2])  # 거리에 따라 정렬
                closest_person = valid_persons[0]

                # 만약 처음 감지된 사람 또는 마지막으로 감지된 사람과 동일한 사람이라면 추적
                if not self.person_detected or (self.last_known_position and self.is_same_person(closest_person, self.last_known_position)):
                    self.person_position = closest_person
                    self.person_detected = True
                    self.last_known_position = closest_person  # 마지막으로 감지된 위치 업데이트
                    self.last_detection_time = current_time  # 마지막 감지 시간 업데이트
                    self.rotation_start_time = None  # 회전 상태 초기화
                else:
                    self.get_logger().info("Detected person does not match the last known person.")
            else:
                self.get_logger().info("No valid persons detected.")


        else:
            # 사람이 감지되지 않았을 때, 마지막 감지 시간으로부터 일정 시간이 경과하면 추적 중단
            if current_time - self.last_detection_time > self.detection_timeout:
                self.person_detected = False  # 추적 상태를 해제
                if self.rotation_start_time is None:
                    self.rotation_start_time = current_time  # 회전 시작 시간 설정
                self.get_logger().info("Person lost. Initiating search by rotation.")
        
        if self.person_detected and self.person_position:
            # 감지된 사람이 있을 경우 로봇의 움직임 조정
            x, y, z = self.person_position
            
            # 로봇의 움직임을 위한 Twist 메시지 생성
            twist = Twist()
            
            # 거리 오차 계산
            distance_error = z - self.target_distance
            
            # 거리 오차에 따라 직진 속도 조정
            twist.linear.x = np.clip(distance_error * 0.5, -self.max_linear_speed, self.max_linear_speed)
            
            # x 위치에 따라 각속도 조정 (방향)
            if x > 0.1:
                twist.angular.z = -min(x * 0.5, self.max_angular_speed)  # 오른쪽 회전
            elif x < -0.1:
                twist.angular.z = -max(x * 0.5, -self.max_angular_speed)  # 왼쪽 회전
            else:
                twist.angular.z = 0.0
            
            # Twist 메시지를 퍼블리시하여 로봇 제어
            self.cmd_vel_publisher.publish(twist)
            
            # 디버깅을 위한 로봇 명령 로그 출력
            self.get_logger().info(f"Publishing cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
        elif not self.person_detected and self.rotation_start_time is not None:
            # 사람이 감지되지 않았고 회전이 시작된 경우
            twist = Twist()
            twist.angular.z = self.max_angular_speed  # 최대 각속도로 회전
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("Rotating to search for person.")
        else:
            # 사람이 감지되지 않았고, 회전이 시작되지 않았으며, 회전 시도가 끝났을 경우 로봇 정지
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("No person detected. Stopping the robot.")
            self.reset_tracking()

def main(args=None):
    rclpy.init(args=args)
    local_node = final_human_tracking()
    rclpy.spin(local_node)
    local_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()