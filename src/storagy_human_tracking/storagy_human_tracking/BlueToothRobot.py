# BlueToothRobot.py
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import time
from std_msgs.msg import Bool
from pyzbar import pyzbar
import bluetooth

class BlueToothRobot(Node):
    def __init__(self):
        super().__init__('BlueToothRobot')

        # 카메라 구독자 설정
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

        # 로봇 제어 퍼블리셔 설정
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 기능 활성화 신호 구독자 설정
        self.activation_subscriber = self.create_subscription(
            Bool,
            '/activation_signal',
            self.activation_callback,
            10
        )

        # QR 코드 인식을 위한 퍼블리셔 설정
        self.qr_code_publisher = self.create_publisher(Bool, '/activation_signal', 10)

        # 유틸리티 초기화
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.depth_image = None
        self.latest_rgb_image = None

        # 상태 변수들
        self.person_detected = False
        self.person_position = None
        self.last_known_position = None
        self.last_detection_time = time.time()
        self.is_active = False
        self.rotation_start_time = None
        self.qr_code_person_position = None  # QR 코드 보여준 사람의 위치

        # 카메라 및 로봇 파라미터 설정
        self.image_width = 640
        self.image_height = 480
        self.fov_x = 90.0
        self.fov_y = 60.0
        self.pixel_to_meter_x = 2 * np.tan(np.radians(self.fov_x / 2)) / self.image_width
        self.pixel_to_meter_y = 2 * np.tan(np.radians(self.fov_y / 2)) / self.image_height
        self.target_distance = 0.4
        self.max_linear_speed = 0.175
        self.max_angular_speed = 0.175

        # 타이머 설정
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.detection_timeout = 3.0

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_qr_code_and_publish(self.latest_rgb_image)

    def activation_callback(self, msg):
        self.is_active = msg.data
        if not self.is_active:
            self.get_logger().info("Person tracking deactivated.")
            self.reset_tracking()

    def reset_tracking(self):
        self.person_detected = False
        self.qr_code_person_position = None
        self.last_known_position = None
        self.rotation_start_time = None

    def detect_qr_code_and_publish(self, image):
        decoded_objects = pyzbar.decode(image)
        for obj in decoded_objects:
            qr_data = obj.data.decode("utf-8")
            self.get_logger().info(f"QR code detected: {qr_data}")
            if qr_data == "START_TRACKING":
                self.qr_code_publisher.publish(Bool(data=True))
                self.is_active = True
            elif qr_data == "STOP_TRACKING":
                self.qr_code_publisher.publish(Bool(data=False))
                self.is_active = False

            self.qr_code_person_position = self.get_qr_code_person_position(image)

    def get_qr_code_person_position(self, image):
        results = self.model.predict(source=image)
        detected_persons = []

        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0:
                    x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                    x_center = int((x1 + x2) / 2)
                    y_center = int((y1 + y2) / 2)

                    if 0 <= x_center < self.image_width and 0 <= y_center < self.image_height:
                        distance = self.depth_image[y_center, x_center] / 1000.0
                        if not np.isnan(distance) and distance > 0:
                            x_center_m = (x_center - self.image_width / 2) * distance * self.pixel_to_meter_x
                            y_center_m = (y_center - self.image_height / 2) * distance * self.pixel_to_meter_y
                            detected_persons.append((x_center_m, y_center_m, distance))

        return detected_persons[0] if detected_persons else None

    def is_same_person(self, current_position, last_position, threshold=0.5):
        distance = np.linalg.norm(np.array(current_position) - np.array(last_position))
        return distance <= threshold

    def timer_callback(self):
        if not self.is_active:
            self.stop_robot()
            return
        
        current_time = time.time()
        if self.latest_rgb_image is None or self.depth_image is None:
            self.get_logger().info("Waiting for images...")
            return
        
        rgb_image = self.latest_rgb_image
        results = self.model.predict(source=rgb_image)
        detected_persons = []

        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0:
                    x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                    x_center = int((x1 + x2) / 2)
                    y_center = int((y1 + y2) / 2)
                    
                    if 0 <= x_center < self.image_width and 0 <= y_center < self.image_height:
                        distance = self.depth_image[y_center, x_center] / 1000.0
                        if not np.isnan(distance) and distance > 0:
                            x_center_m = (x_center - self.image_width / 2) * distance * self.pixel_to_meter_x
              
                            y_center_m = (y_center - self.image_height / 2) * distance * self.pixel_to_meter_y
                     
                     
                            detected_persons.append((x_center_m, y_center_m, distance))
        min_detection_distance = 0.5  # 최소 감지 거리 설정 (예: 1 미터)

        if self.qr_code_person_position is not None:
            closest_person = self.qr_code_person_position
            if closest_person and (not self.person_detected or self.is_same_person(closest_person, self.last_known_position)):
                self.person_position = closest_person
                self.person_detected = True
                self.last_known_position = closest_person
                self.last_detection_time = current_time
                self.rotation_start_time = None
            else:
                self.get_logger().info("Detected person does not match the QR code person.")
        else:
            if detected_persons:
                closest_person = min(detected_persons, key=lambda p: p[2])
                if closest_person[2] < min_detection_distance:  # 거리가 너무 멀면 무시

                    if not self.person_detected or self.is_same_person(closest_person, self.last_known_position):
                        self.person_position = closest_person
                        self.person_detected = True
                        self.last_known_position = closest_person
                        self.last_detection_time = current_time
                        self.rotation_start_time = None
                    else:
                        self.get_logger().info("Detected person does not match the last known person.")
            else:
                if current_time - self.last_detection_time > self.detection_timeout:
                    self.get_logger().info("No person detected. Stopping the robot.")
                    self.stop_robot()
                    return

        if self.person_detected:
            self.move_to_person(self.person_position)

    def move_to_person(self, person_position):
        twist = Twist()
        x, y, _ = person_position
        linear_speed = min(self.max_linear_speed, self.target_distance / np.linalg.norm([x, y]) * self.max_linear_speed)
        twist.linear.x = linear_speed

        # Calculate angular velocity
        if x != 0:
            twist.angular.z = np.clip(np.arctan2(y, x), -self.max_angular_speed, self.max_angular_speed)
        else:
            twist.angular.z = 0

        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    local_node = BlueToothRobot()
    rclpy.spin(local_node)
    local_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
