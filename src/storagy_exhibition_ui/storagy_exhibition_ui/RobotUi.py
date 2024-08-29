import sys
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget, QMainWindow
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
import rclpy
from rclpy.node import Node
from Parameter_msgs.srv import Cam
import time 

class ROS2Node(Node):
    def __init__(self):
        super().__init__('pyqt5_ros2_node')
        self.get_logger().info("ROS 2 Node started")
        #qr 카메라를 키기 위한 서비스 클라이언트 만들기 
        self.client = self.create_client(Cam,'cam_on')
        #cam키기 요청함수    
    
    def send_request(self,cam):
        request = Cam.Request()
        request.camera_qr = cam
        # 응답을 처리하지 않고 요청만 보냄
        self.future = self.client.call_async(request)
        self.get_logger().info('Request sent, not waiting for response.')

# QR - ID 등록창 
class FourthWindow(QWidget):
    def __init__(self, next_window, ros_node):
        super().__init__()

        self.setWindowTitle("QR등록")
        self.setGeometry(150, 150, 1124, 632)
        
        self.next_window = next_window  # 다음 창을 저장
        self.ros_node = ros_node  # ROS 2 노드 저장

        # 배경 이미지 추가
        self.background_label = QLabel(self)
        pixmap = QPixmap("/home/sineunji/ros2_ws/src/storagy_exhibition_ui/UIpiture/스크린샷 2024-08-11 17-14-50.png")
        self.background_label.setPixmap(pixmap)
        self.background_label.setScaledContents(True)  # 이미지가 QLabel 크기에 맞게 조정되도록 설정
        self.background_label.setGeometry(0, 0, 1124, 632)  # QLabel 크기를 창 크기에 맞게 설정

        # QR 버튼 -- ID 등록
        self.qr_button = QPushButton("QR", self)
        self.qr_button.setFixedSize(100, 100)  # 버튼 크기 설정 (너비, 높이)
        self.qr_button.move(505, 270)  # 중앙 정렬
        self.qr_button.clicked.connect(self.scan_on)  # 연결 
        
        # 'Next' 버튼
        self.next_button = QPushButton("Next", self)
        self.next_button.setFixedSize(1100, 20)  # 세 번째 창과 동일한 크기 설정
        self.next_button.move(5, 600)  # 세 번째 창과 동일한 위치 설정
        self.next_button.clicked.connect(self.show_next_window)
    
    # scan on QR인식을 위한 카메라 on 10초간 켜지기
    def scan_on(self):
        self.ros_node.send_request(True) 
    
    
    def send_request(self,cam):
        request = Cam.Request()
        request.camera_qr = cam
        # 응답을 처리하지 않고 요청만 보냄
        self.future = self.client.call_async(request)
        self.get_logger().info('Request sent, not waiting for response.')

    # 네 번째 창을 닫음
    def show_next_window(self):
        self.next_window.show()
        self.close()

# 주의사항
class FifthWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fifth Window")
        self.setGeometry(150, 150, 1120, 629)
        
        layout = QVBoxLayout()
        
        # 배경 이미지 추가
        self.label = QLabel(self)
        pixmap = QPixmap("/home/sineunji/ros2_ws/src/storagy_exhibition_ui/UIpiture/스크린샷 2024-08-11 16-42-06.png")
        self.label.setPixmap(pixmap)
        self.label.setScaledContents(True)  # 이미지가 QLabel 크기에 맞게 조정되도록 설정
        self.label.setGeometry(0, 0, 1120, 629)  # QLabel 크기를 창 크기에 맞게 설정
        
        layout.addWidget(self.label)
        
        self.setLayout(layout)
        
#메인 맵     
class MyApp(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
                
        self.setWindowTitle("Simple PyQt5 UI")
        self.setGeometry(0, 0, 1915, 1069)
        
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        
        # QLabel에 배경 이미지 설정
        self.label = QLabel(self)
        pixmap = QPixmap("/home/sineunji/ros2_ws/src/storagy_exhibition_ui/UIpiture/스크린샷_2024-08-11_resized.png")
        self.label.setPixmap(pixmap)
        self.label.setGeometry(0, 0, 1915, 1069)
        
        # 첫 번째 버튼
        self.button = QPushButton("지도보기", self)
        self.button.setGeometry(850, 800, 200, 50)  # x, y, width, height
        self.button.clicked.connect(self.show_second_window)

        # 두 번째 버튼
        self.new_button = QPushButton("로봇사용", self)
        self.new_button.setGeometry(850, 850, 200, 50)  # x, y, width, height
        self.new_button.clicked.connect(self.show_robot_window)

        # 추가적인 윈도우 설정
        self.second_window = SecondWindow()
        self.fifth_window = FifthWindow()  # 다섯 번째 창 생성
        self.fourth_window = FourthWindow(self.fifth_window, ros_node)  # 다섯 번째 창을 네 번째 창에 전달
        self.third_window = ThirdWindow(self.fourth_window)
        self.robot_window = RobotWindow(self.third_window)

    def show_second_window(self):
        self.second_window.show()

    def show_robot_window(self):
        self.robot_window.show()
# 로봇 사용 
class SecondWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Second Window")
        self.setGeometry(150, 150, 1120, 629)

        layout = QVBoxLayout()
        self.label = QLabel(self)
        pixmap = QPixmap("/home/sineunji/ros2_ws/src/storagy_exhibition_ui/UIpiture/스크린샷 2024-08-11 18-39-33.png")
        scaled_pixmap = pixmap.scaled(1120, 629, Qt.KeepAspectRatio)
        self.label.setPixmap(scaled_pixmap)
        self.label.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(self.label)
        self.setLayout(layout)

class ThirdWindow(QWidget):
    def __init__(self, next_window):
        super().__init__()

        self.next_window = next_window

        self.setWindowTitle("Third Window")
        self.setGeometry(150, 150, 1120, 629)
        
        layout = QVBoxLayout()
        self.label = QLabel(self)
        pixmap = QPixmap("/home/sineunji/ros2_ws/src/storagy_exhibition_ui/UIpiture/스크린샷 2024-08-13 15-55-28.png")
        scaled_pixmap = pixmap.scaled(1120, 629, Qt.KeepAspectRatio)
        self.label.setPixmap(scaled_pixmap)
        self.label.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(self.label)
        self.next_button = QPushButton("next", self)
        self.next_button.clicked.connect(self.on_next_clicked)
        layout.addWidget(self.next_button)

        self.setLayout(layout)

    def on_next_clicked(self):
        self.close()
        self.next_window.show()
#로봇 사용 
class RobotWindow(QWidget):
    def __init__(self, next_window):
        super().__init__()

        self.next_window = next_window

        self.setWindowTitle("Robot Window")
        self.setGeometry(150, 150, 1120, 629)

        self.setStyleSheet("background-image: url('/home/sineunji/ros2_ws/src/storagy_exhibition_ui/UIpiture/스크린샷 2024-08-11 16-42-01.png');")  
        
        layout = QVBoxLayout()
        self.label = QLabel("", self)
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)

        self.button = QPushButton("next", self)
        self.button.clicked.connect(self.show_next_window)
        layout.addWidget(self.button)
        
        self.setLayout(layout)

    def show_next_window(self):
        self.close()
        self.next_window.show()


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    ros_node = ROS2Node()  # ROS 2 노드 생성
    app = QApplication(sys.argv)
    window = MyApp(ros_node)
    window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()
