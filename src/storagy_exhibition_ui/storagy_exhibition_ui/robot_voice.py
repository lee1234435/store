import rclpy
from rclpy.node import Node
from playsound import playsound
from std_srvs.srv import Empty
from PyQt5.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget
import sys
import threading

class SirenNode(Node):
    def __init__(self):
        super().__init__('siren_node')
        self.get_logger().info('Siren Node has started.')
        self.cli = self.create_client(Empty, '/emergency')
         
        # 서비스가 사용 가능할 때까지 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
    # 서비스 요청 보내기
    def send_request(self):
        self.req = Empty.Request()
        # 사이렌 재생을 비동기적으로 처리
        threading.Thread(target=self.play_siren, daemon=True).start()
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.handle_response)

    # 사이렌 소리 재생
    def play_siren(self):
        try:
            file_path = '/home/sineunji/ros2_ws/src/storagy_exhibition_ui/storagy_ui/siren.mp3'
            playsound(file_path)
            playsound(file_path)
            self.get_logger().info('Playing siren sound.')
        except Exception as e:
            self.get_logger().error(f'Error playing sound: {e}')

    # 서비스 응답 처리
    def handle_response(self, future):
        try:
            future.result()  # 서비스 호출 결과를 기다림 (Empty 타입이므로 반환값 없음)
            self.get_logger().info('Service call succeeded.')
        except Exception as e:
            self.get_logger().error(f'Error processing service response: {e}')

class SirenGUI(QWidget):
    def __init__(self, siren_node):
        super().__init__()
        self.siren_node = siren_node
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Siren Control')
        button = QPushButton('Activate Siren', self)
        button.clicked.connect(self.on_button_clicked)
        layout = QVBoxLayout()
        layout.addWidget(button)
        self.setLayout(layout)

    def on_button_clicked(self):
        self.siren_node.send_request()

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    siren_node = SirenNode()

    # Qt 애플리케이션 초기화
    app = QApplication(sys.argv)

    # GUI 생성 및 표시
    gui = SirenGUI(siren_node)
    gui.show()

    # ROS 스레드를 생성하고 시작
    ros_thread = threading.Thread(target=ros_spin_thread, args=(siren_node,))
    ros_thread.start()
    
    # Qt 애플리케이션 실행
    result = app.exec_()

    # ROS 종료
    rclpy.shutdown()
    ros_thread.join()  # 스레드가 종료되기를 기다림

    sys.exit(result)

if __name__ == '__main__':
    main()





