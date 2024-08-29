import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np
from std_msgs.msg import String 
import tf2_ros
import math  
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R  # 회전 행렬을 쿼터니언으로 변환
import yaml
from std_msgs.msg import Int32

class ApriltagDetectionNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')

        # TF Buffer와 Listener 초기화 (ROS 좌표 변환을 위한 도구)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 카메라 이미지 토픽 구독 (Color 이미지)
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_image_callback,
            10)

        # Depth 카메라 이미지 토픽 구독 (Depth 이미지)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10)

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',  # 실제 사용 중인 IMU 토픽 이름으로 변경
            self.imu_callback,
            10)

        # AMCL로부터 피드백을 받기 위한 토픽 구독
        self.feedback_subscription = self.create_subscription(
            String,
            'apriltag_feedback',
            self.feedback_callback,
            10
            )

        self.bridge = CvBridge()
        self.detector = apriltag.apriltag("tag36h11")
        
        # 이미지 및 위치 퍼블리셔 생성
        self.image_pub = self.create_publisher(Image, '/apriltag_detection_image', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'apriltag_pose', 10)
        self.current_orientation = None
        self.color_frame = None
        self.depth_frame = None
        self.imu_yaw = None 
        #데이터 보내기 퍼블리셔 
        self.tag_pub = self.create_publisher(Int32, 'coord_data', 10)
        # 태그 ID에 대한 월드 좌표 설정 (Map 좌표계 기준)
        self.tag_coordinates = {
                0: (4.987993240, -1.04322886, 0.32),
                1: (4.546353340148926, 1.1332958936691284, 0.61),
                2: (3.42607712, -0.445754587, 0.55),
                3: (1.4073152542, 0.645906567, 0.27),
                4: (3.2566099166870117, -2.4146056175231934, 0.28),
                5: (3.002598, -2.4304530, 0.29),
                6: (2.8743224143, 1.7236061096, 0.56),
                7: (4.74539375, 0.558673858, 0.32),
                8: (5.55115229, 0.275051201, 0.53),
                9: (4.7196235, -2.5375328, 0.30),
                10: (2.303166627, 1.28135883808, 0.27),
                11: (2.0105538368, -2.5801603794, 0.275),
                12: (3.857791, -2.4696977, 0.26),
                13: (4.226512908935547, -2.4479730129241943, 0.52),
                #14: (2.4429757595062256, -2.341637134552002, 0.57),
                15: (5.529539108276367, -0.2425769716501236, 0.28),
                16: (2.084618091583252, 1.154811143875122, 0.32),
                17: (2.3057069778442383, 1.5482909679412842, 0.23),
                18: (1.478023886680603, 0.9653726816177368, 0.23),
                19: (1.7643160820007324, 1.1299530267715454, 0.54),
                20: (4.258351802825928, 1.7059189081192017, 0.27),
                21: (3.8528831005096436, 1.7191468477249146, 0.61),
                22: (3.4360427856445312, 1.7011438608169556, 0.28),
                23: (5.220964431762695, 0.5418509244918823, 0.32),
                24: (4.613394260406494, 0.8273558020591736, 0.36),
                25: (4.511187553405762, 1.3130862712860107, 0.36)
                
                }

        # 타이머 설정: 3초마다 process_images 함수 호출
        self.timer = self.create_timer(3.0, self.process_images)

        # Calibration data 읽기
        self.load_calibration_data('/home/storagy/Desktop/PINKLAB_SONG/src/storagy_apriltag2AMCL/calibration/calibration_data.yaml')
            

    def load_calibration_data(self, filename):
        with open(filename, 'r') as f:
            calibration_data = yaml.safe_load(f)
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])
        self.get_logger().info(f"Loaded calibration data from {filename}")

    def undistort_image(self, image):
        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        undistorted_image = cv2.undistort(image, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        return undistorted_image

    def color_image_callback(self, msg):
        # 카메라 이미지 콜백 함수
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 이미지를 왜곡 보정
        self.color_frame = self.undistort_image(self.color_frame)
        
    def depth_image_callback(self, msg):
        # Depth 카메라 이미지 콜백 함수
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def feedback_callback(self, msg):
        # AMCL로부터의 피드백 처리
        self.get_logger().info(f"Received feedback from AMCL: {msg.data}")

    def imu_callback(self, msg):
        # IMU 메시지에서 오리엔테이션(quaternion)을 저장
        self.current_orientation = msg.orientation

        # Quaternion을 Euler angles (roll, pitch, yaw)로 변환
        _, _, yaw = quat2euler([self.current_orientation.x,
                                self.current_orientation.y,
                                self.current_orientation.z,
                                self.current_orientation.w])
        self.get_logger().info(f"IMU yaw: {yaw}")
        self.imu_yaw = yaw

    def fuse_yaw(self, apriltag_yaw, imu_yaw):
        """
        IMU data 와 계산값 가중치 적용
        """
        # 가중치를 정하여 IMU와 AprilTag 데이터를 융합
        alpha = 0.7  # IMU에 대한 가중치
        beta = 1.0 - alpha  # AprilTag에 대한 가중치

        # 가중 평균으로 yaw 값을 융합
        fused_yaw = alpha * imu_yaw + beta * apriltag_yaw
        return fused_yaw


    def process_images(self):
        """
        카메라 토픽 구독 후 코드 구동
        """
        # Color 또는 Depth 프레임이 없을 경우 처리 생략
        if self.color_frame is None or self.depth_frame is None:
            self.get_logger().info("Color frame or depth frame is missing. Skipping processing.")
            return

        self.get_logger().info("Processing images...")

        # Gray 이미지로 변환 후 AprilTag 검출
        gray_frame = cv2.cvtColor(self.color_frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray_frame)

        MIN_AREA = 2000  # 최소 면적 기준 설정
        MIN_DEPTH = 0.1  # 최소 신뢰할 수 있는 Depth 값 (0.1m)

        if len(detections) > 0:
            self.get_logger().info(f"Detected {len(detections)} AprilTags.")
            
            # 각 태그의 ID와 면적을 로깅
            valid_detections = []
            for detection in detections:
                tag_id = detection['id']
                corners = np.array(detection['lb-rb-rt-lt']).astype(np.int32)
                area = cv2.contourArea(corners)
                self.get_logger().info(f"Tag ID: {tag_id}, Area: {area}")

                # 중심점 계산
                center = np.mean(corners, axis=0).astype(int)
                
                # 인덱스 범위 확인
                height, width = self.depth_frame.shape
                if 0 <= center[0] < width and 0 <= center[1] < height:
                    depth_value = self.depth_frame[center[1], center[0]] / 1000.0  # Depth 값을 미터 단위로 변환
                else:
                    self.get_logger().warn(f"Center coordinates out of bounds: {center}")
                    continue  # 잘못된 좌표는 무시하고 다음 태그로 넘어감

                # Depth 값이 너무 작은 경우 태그 무시
                if depth_value < MIN_DEPTH or depth_value < 0:
                    self.get_logger().warn(f"Depth value for Tag ID {tag_id} is too small ({depth_value}m). Skipping this tag.")
                    continue  # 신뢰할 수 없는 태그를 무시하고 다음으로 넘어감

                valid_detections.append(detection)

            # 최소 면적 이상의 태그만 필터링
            filtered_detections = [d for d in valid_detections if cv2.contourArea(np.array(d['lb-rb-rt-lt']).astype(np.int32)) >= MIN_AREA]
            
            #최소 면적 이상의 태그 id 송신, 태그 데이터 보내기 --> msg32 보냄
            msg32 = Int32()
            #따로 복사본 만듬
            filtered_detections2 = filtered_detections.copy()
            if filtered_detections2:
               largest_detection2 = filtered_detections2[0]
               filtered_detections2.sort(key=lambda d: cv2.contourArea(np.array(d['lb-rb-rt-lt']).astype(np.int32)), reverse=True)
               largest_detection2 = filtered_detections2[0]
               send_tag = largest_detection2['id']
               msg32.data = send_tag
               self.tag_pub.publish(msg32)
               self.get_logger().info(f'데이터 Tag의값 {msg32.data}')

           # 이후의 로직을 여기에 추가
            else:
               # 처리할 태그가 없는 경우에 대한 예외 처리
                self.get_logger().info("No valid detections found. Skipping...")
             
            if len(filtered_detections) < 3:
                self.get_logger().info("Not enough valid tags above the minimum area and depth threshold. Need at least three.")
                return
            
            # 가장 큰 세 태그 검출
            filtered_detections.sort(key=lambda d: cv2.contourArea(np.array(d['lb-rb-rt-lt']).astype(np.int32)), reverse=True)
            largest_detection = filtered_detections[0]
            second_largest_detection = filtered_detections[1]
            third_largest_detection = filtered_detections[2]

            self.get_logger().info(f"Largest detection ID: {largest_detection['id']}, Area: {cv2.contourArea(np.array(largest_detection['lb-rb-rt-lt']).astype(np.int32))}")
            self.get_logger().info(f"Second Largest detection ID: {second_largest_detection['id']}, Area: {cv2.contourArea(np.array(second_largest_detection['lb-rb-rt-lt']).astype(np.int32))}")
            self.get_logger().info(f"Third Largest detection ID: {third_largest_detection['id']}, Area: {cv2.contourArea(np.array(third_largest_detection['lb-rb-rt-lt']).astype(np.int32))}")

            # 태그 ID에 해당하는 월드 좌표를 가져옴
            tag_id_1 = largest_detection['id']
            tag_id_2 = second_largest_detection['id']
            tag_id_3 = third_largest_detection['id']
           
            #태그 데이터 보내기 --> msg32 보냄
            msg32 = Int32()
            msg32.data = tag_id_1
            if msg32.data is not None :
               self.tag_pub.publish(msg32)
             
            if tag_id_1 in self.tag_coordinates and tag_id_2 in self.tag_coordinates and tag_id_3 in self.tag_coordinates:
                world_position_1 = np.array(self.tag_coordinates[tag_id_1])
                world_position_2 = np.array(self.tag_coordinates[tag_id_2])
                world_position_3 = np.array(self.tag_coordinates[tag_id_3])
                self.get_logger().info(f"World Position 1: {world_position_1}, World Position 2: {world_position_2}, World Position 3: {world_position_3}")
            else:
                self.get_logger().warn("One or more tag IDs are not in the world coordinates map.")
                return

            # 월드 좌표계에서의 태그 간 각도 계산
            theta_12 = np.arctan2(world_position_2[1] - world_position_1[1], 
                                world_position_2[0] - world_position_1[0])
            theta_13 = np.arctan2(world_position_3[1] - world_position_1[1], 
                                world_position_3[0] - world_position_1[0])
            theta_23 = np.arctan2(world_position_3[1] - world_position_2[1], 
                                world_position_3[0] - world_position_2[0])
            self.get_logger().info(f"Calculated theta_12: {theta_12}, theta_13: {theta_13}, theta_23: {theta_23}")

            # 카메라 상대 위치 계산
            corners_1 = np.array(largest_detection['lb-rb-rt-lt']).astype(np.int32)
            corners_2 = np.array(second_largest_detection['lb-rb-rt-lt']).astype(np.int32)
            corners_3 = np.array(third_largest_detection['lb-rb-rt-lt']).astype(np.int32)
            center_1 = np.mean(corners_1, axis=0).astype(int)
            center_2 = np.mean(corners_2, axis=0).astype(int)
            center_3 = np.mean(corners_3, axis=0).astype(int)
            depth_value_1 = self.depth_frame[center_1[1], center_1[0]] / 1000.0
            depth_value_2 = self.depth_frame[center_2[1], center_2[0]] / 1000.0
            depth_value_3 = self.depth_frame[center_3[1], center_3[0]] / 1000.0
            relative_camera_position_1 = self.calculate_relative_position(depth_value_1, corners_1)
            self.get_logger().info(f"Estimated relative_camera_position from Tag ID {tag_id_1}: {relative_camera_position_1}")
            relative_camera_position_2 = self.calculate_relative_position(depth_value_2, corners_2)
            self.get_logger().info(f"Estimated relative_camera_position from Tag ID {tag_id_2}: {relative_camera_position_2}")
            relative_camera_position_3 = self.calculate_relative_position(depth_value_3, corners_3)
            self.get_logger().info(f"Estimated relative_camera_position from Tag ID {tag_id_3}: {relative_camera_position_3}")

            # 카메라 기준 상대적인 각도 계산
            relative_angle_alpha_12 = np.arctan2(relative_camera_position_2[1] - relative_camera_position_1[1], 
                                            relative_camera_position_2[0] - relative_camera_position_1[0])
            relative_angle_alpha_13 = np.arctan2(relative_camera_position_3[1] - relative_camera_position_1[1], 
                                            relative_camera_position_3[0] - relative_camera_position_1[0])
            relative_angle_alpha_23 = np.arctan2(relative_camera_position_3[1] - relative_camera_position_2[1], 
                                            relative_camera_position_3[0] - relative_camera_position_2[0])
            self.get_logger().info(f"Calculated relative_angle_alpha_12: {relative_angle_alpha_12}, relative_angle_alpha_13: {relative_angle_alpha_13}, relative_angle_alpha_23: {relative_angle_alpha_23}")

            # AprilTag를 사용하여 계산된 yaw
            yaw_from_apriltag_12 = theta_12 - relative_angle_alpha_12
            yaw_from_apriltag_13 = theta_13 - relative_angle_alpha_13
            yaw_from_apriltag_23 = theta_23 - relative_angle_alpha_23
            self.get_logger().info(f"Yaw from AprilTag 12: {yaw_from_apriltag_12}, Yaw from AprilTag 13: {yaw_from_apriltag_13}, Yaw from AprilTag 23: {yaw_from_apriltag_23}")

            # 여러 태그를 사용한 yaw의 평균 계산
            yaw_from_apriltag = np.mean([yaw_from_apriltag_12, yaw_from_apriltag_13, yaw_from_apriltag_23])

            # IMU와 AprilTag yaw 융합
            if self.imu_yaw is not None:
                yaw = self.fuse_yaw(yaw_from_apriltag, self.imu_yaw)
                self.get_logger().info(f"Fused Yaw: {yaw}")
            else:
                yaw = yaw_from_apriltag  # IMU 데이터가 없으면 AprilTag yaw만 사용

            # 카메라 상대 위치를 base_link 좌표계로 변환
            relative_position_1_in_base = relative_camera_position_1 - np.array([0.198, 0.0, 0.2295])
            self.get_logger().info(f"Estimated robot relative position from Tag ID {tag_id_1}: {relative_position_1_in_base}")
            relative_position_2_in_base = relative_camera_position_2 - np.array([0.198, 0.0, 0.2295])
            self.get_logger().info(f"Estimated robot relative position from Tag ID {tag_id_2}: {relative_position_2_in_base}")
            relative_position_3_in_base = relative_camera_position_3 - np.array([0.198, 0.0, 0.2295])
            self.get_logger().info(f"Estimated robot relative position from Tag ID {tag_id_3}: {relative_position_3_in_base}")

            # base_link 좌표계에서 월드 좌표계로 변환
            world_position_1_in_world = self.transform_relative_to_world(world_position_1, relative_position_1_in_base, yaw)
            self.get_logger().info(f"Estimated robot position from Tag ID {tag_id_1}: {world_position_1_in_world}")
            world_position_2_in_world = self.transform_relative_to_world(world_position_2, relative_position_2_in_base, yaw)
            self.get_logger().info(f"Estimated robot position from Tag ID {tag_id_2}: {world_position_2_in_world}")
            world_position_3_in_world = self.transform_relative_to_world(world_position_3, relative_position_3_in_base, yaw)
            self.get_logger().info(f"Estimated robot position from Tag ID {tag_id_3}: {world_position_3_in_world}")

            # 보정된 최종 위치 계산
            final_position = (world_position_1_in_world + world_position_2_in_world + world_position_3_in_world) / 3.0
            self.get_logger().info(f"Final calculated position (corrected): {final_position}")

            final_position[1] += 0.3
            self.get_logger().info(f"Final calculated position after y correction: {final_position}")
            
            # Orientation은 이미 계산된 yaw를 사용하여 생성
            orientation = R.from_euler('z', yaw).as_quat()
            self.get_logger().info(f"Calculated orientation (quaternion): {orientation}")

            # 최종 포즈 생성 및 퍼블리시
            pose_msg = self.create_pose_stamped(final_position, orientation)

            if pose_msg:
                self.get_logger().info(f"Publishing pose to apriltag_pose topic.")
                self.pose_publisher.publish(pose_msg)
            else:
                self.get_logger().warn("Pose message is None. Skipping publishing.")
            
            # 검출된 태그의 윤곽선과 정보를 이미지에 표시
            cv2.polylines(self.color_frame, [corners_1], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.putText(self.color_frame, f"ID: {tag_id_1}", (center_1[0], center_1[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(self.color_frame, f"Coordinates: {final_position}", (center_1[0], center_1[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(self.color_frame, tuple(center_1), 5, (0, 255, 0), -1)

            cv2.polylines(self.color_frame, [corners_2], isClosed=True, color=(255, 0, 0), thickness=2)
            cv2.putText(self.color_frame, f"ID: {tag_id_2}", (center_2[0], center_2[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.circle(self.color_frame, tuple(center_2), 5, (255, 0, 0), -1)

            cv2.polylines(self.color_frame, [corners_3], isClosed=True, color=(0, 0, 255), thickness=2)
            cv2.putText(self.color_frame, f"ID: {tag_id_3}", (center_3[0], center_3[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.circle(self.color_frame, tuple(center_3), 5, (0, 0, 255), -1)
            
            # 첫 번째 태그에 대한 Depth 값 로깅
            self.get_logger().info(f"Depth Value for Tag ID {tag_id_1}: {depth_value_1} meters")

            # 두 번째 태그에 대한 Depth 값 로깅
            self.get_logger().info(f"Depth Value for Tag ID {tag_id_2}: {depth_value_2} meters")

            # 세 번째 태그에 대한 Depth 값 로깅
            self.get_logger().info(f"Depth Value for Tag ID {tag_id_3}: {depth_value_3} meters")

            # 첫 번째 태그로부터 로봇 베이스 기준 위치 추정 및 로깅
            self.get_logger().info(f"Relative Position 1 in Base Link (before world transform): {relative_position_1_in_base}")

            # 첫 번째 태그로부터 월드 좌표로 변환된 로봇 위치 로깅
            self.get_logger().info(f"Estimated robot position from Tag ID {tag_id_1} (world coordinates): {world_position_1_in_world}")

            # 두 번째 태그로부터 로봇 베이스 기준 위치 추정 및 로깅
            self.get_logger().info(f"Relative Position 2 in Base Link (before world transform): {relative_position_2_in_base}")

            # 두 번째 태그로부터 월드 좌표로 변환된 로봇 위치 로깅
            self.get_logger().info(f"Estimated robot position from Tag ID {tag_id_2} (world coordinates): {world_position_2_in_world}")

            # 세 번째 태그로부터 로봇 베이스 기준 위치 추정 및 로깅
            self.get_logger().info(f"Relative Position 3 in Base Link (before world transform): {relative_position_3_in_base}")

            # 세 번째 태그로부터 월드 좌표로 변환된 로봇 위치 로깅
            self.get_logger().info(f"Estimated robot position from Tag ID {tag_id_3} (world coordinates): {world_position_3_in_world}")

            # 최종적으로 처리된 이미지를 퍼블리시
            processed_image_msg = self.bridge.cv2_to_imgmsg(self.color_frame, encoding='bgr8')
            self.get_logger().info("Publishing to /apriltag_detection_image") 
            self.image_pub.publish(processed_image_msg)

   
    def transform_relative_to_world(self, world_position, relative_position, yaw):
        """
        ROBOT base link --> World 
        """
        # 로봇의 기본 좌표계에서 카메라의 위치 (평행 이동 벡터)
        # 로봇의 yaw에 따른 회전 행렬 계산
        rotation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0,           0,           1]
        ])
        self.get_logger().info(f"base to world: {np.dot(rotation_matrix, relative_position )}")
        # 상대적인 카메라 위치를 월드 좌표계로 변환
        world_position_adjusted = world_position - np.dot(rotation_matrix, relative_position )
        return world_position_adjusted

    def calculate_relative_position(self, depth_value, corners):
        """
        로봇과 태그간의 상대 위치 계산
        """
        # 코너의 중앙값을 이용해 각도를 계산합니다.
        vector_to_tag = np.mean(corners, axis=0)
        
        # 카메라 매트릭스에서 focal length를 얻습니다.
        fx = self.camera_matrix[0][0]  # fx
        fy = self.camera_matrix[1][1]  # fy
        cx = self.camera_matrix[0][2]  # cx (principal point x)
        cy = self.camera_matrix[1][2]  # cy (principal point y)

        # 태그 중심이 중심에서 떨어진 픽셀 거리
        delta_x = vector_to_tag[0] - cx
        delta_y = vector_to_tag[1] - cy

        # 실제 거리로 변환 (Z, X, Y 축 순으로 계산)
        z_meters = depth_value  # 깊이 값
        x_meters = (delta_x * z_meters) / fx
        y_meters = (delta_y * z_meters) / fy

        # 상대적인 카메라 위치를 배열로 생성
        relative_position = np.array([x_meters, y_meters, z_meters]).reshape(-1, 1)

        # 카메라의 optical frame 기준으로 변환하는 회전 행렬
        camera_rotation = np.array([
            [0, 0, 1],  # z (optical frame) -> x (robot)
            [-1, 0, 0],  # x (optical frame) -> -y (robot)
            [0, -1, 0]   # y (optical frame) -> -z (robot)
        ])

        # 회전 행렬을 적용하여 카메라 링크 좌표로 변환
        camera_frame_position = np.dot(camera_rotation, relative_position).flatten()

        return camera_frame_position

    def create_pose_stamped(self, position, orientation):
        """
        publish to AMCL
        """
        if position is None:
            self.get_logger().warn("World position is None, skipping pose creation.")
            return None

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # 월드 좌표계인 "map"을 기준으로 설정
        
        pose_msg.pose.position.x = float(position[0]) 
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        
        pose_msg.pose.orientation.x = float(orientation[0])
        pose_msg.pose.orientation.y = float(orientation[1])
        pose_msg.pose.orientation.z = float(orientation[2])
        pose_msg.pose.orientation.w = float(orientation[3])

        return pose_msg


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


