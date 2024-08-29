import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import datetime 
import sqlite3

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'coord_data',
            self.append_tag,
            10)
        # 좌표 태그 
        # 끝은 -1
        # 1. 데이터베이스 연결 (기존 데이터베이스에 연결)
        self.conn = sqlite3.connect('/home/sineunji/ros2_ws/src/storagy_exhibition_ui/database/graph')  # 'example.db'는 기존의 SQLite 데이터베이스 파일입니다.
        self.cursor = self.conn.cursor() 
      
        self.tag_index = []
        self.index_count = 0
        self.label = None
    
    #3초마다 리스트 붙이기 
    def append_tag(self,msg):
        #주행중에 위치 데이터 들어옴
        if msg.data : 
           self.tag_index.append(msg.data) #self.index_count
           self.index_count = self.index_count + 1  
           self.get_logger().info(f'태그 저장중 {msg.data} ..')         
        #주차할때 리스트 저장 끝 신호 --> 자꾸 22번 인식할 경우 가정 
        #   if len(self.tag_index) > 3 and (self.tag_index[self.index_count - 1] == 22 and self.tag_index[self.index_count - 2] == 22):
        #      self.get_logger().info(f'22번 구역에 계속 있는 중 ...')
        #      #계속 주차중이라고 가정 
        #      #reset 
        #      self.tag_index = []
        #      self.index_count = 0
        #      self.label = None   
              
        #      return 
           if self.tag_index[self.index_count - 1] == 21 or self.tag_index[self.index_count - 1] == 6:
               
           
               #인덱스 : 시간대 , 좌표: 태그 id 리스트 
               self.label = str(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
               self.tag_index_str = ','.join(map(str, self.tag_index))
               self.get_logger().info(f'값 저장합니다 {self.tag_index}')
               self.get_logger().info(f'시간 저장합니다 {self.label}')
              #변경사항 저장 
               sql = '''
               INSERT INTO tracking_graph (time, "index")
               VALUES (?, ?)
               '''
               data = (str(self.label),str(self.tag_index))  # 넣을 데이터 
               self.cursor.execute(sql, data) 
               # 3. 변경사항 저장
               self.conn.commit()
               #reset 
               self.tag_index = []
               self.index_count = 0
               self.label = None        
           return 
        
        else:
           self.get_logger().info(f'로봇이 실행하지 않습니다')
           
        
def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



