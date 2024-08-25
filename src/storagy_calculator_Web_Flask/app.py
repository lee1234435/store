# import cv2
# import numpy as np
# from flask import Flask, request, render_template, redirect, url_for, jsonify
# import sqlite3
# from ultralytics import YOLO
# import base64
# import threading
# import time

# app = Flask(__name__)

# # YOLOv8n 모델 로드
# model = YOLO('best.pt')

# def get_db_connection():
#     conn = sqlite3.connect('database.db')
#     conn.row_factory = sqlite3.Row
#     return conn

# def process_image(image):
#     # YOLOv8n 모델을 사용하여 이미지에서 바코드를 인식합니다
#     results = model(image)
#     detected_items = []
    
#     for result in results:
#         for box in result.boxes:
#             x1, y1, x2, y2 = box.xyxy[0]
#             conf = box.conf[0]
#             if conf > 0.5:
#                 # Extract detected item string from the result
#                 class_id = int(box.cls[0].item())  # Convert tensor to integer
#                 detected_item_str = result.names[class_id]
#                 print(f"Detected item string: {detected_item_str}")

#                 # Parse item string
#                 parts = detected_item_str.split('-')
#                 print(f"Parts after split: {parts}")

#                 if len(parts) == 5:
#                     name = parts[0]
#                     price = parts[1]
#                     year = parts[2]
#                     month = parts[3].zfill(2)  # Ensure month is 2 digits
#                     day = parts[4].zfill(2)    # Ensure day is 2 digits
#                     expiry_date = f"{year}-{month}-{day}"
#                     detected_items.append({
#                         'name': name,
#                         'price': int(price),
#                         'expiry_date': expiry_date,
#                         'quantity': 1
#                     })
#                 else:
#                     print(f"Item data is not in the expected format: {detected_item_str}. Error: Unexpected number of parts in the item data")

#     return detected_items

# @app.route('/')
# def index():
#     conn = get_db_connection()
#     items = conn.execute('SELECT * FROM items').fetchall()
#     conn.close()
#     return render_template('index.html', items=items)

# @app.route('/update', methods=['POST'])
# def update_item():
#     name = request.form['name']
#     price = float(request.form['price'])
#     expiry_date = request.form['expiry_date']
#     quantity = int(request.form['quantity'])

#     conn = get_db_connection()
    
#     existing_item = conn.execute('SELECT * FROM items WHERE name = ?', (name,)).fetchone()

#     if existing_item:
#         new_quantity = existing_item['quantity'] + quantity
#         conn.execute('''
#             UPDATE items
#             SET price = ?, expiry_date = ?, quantity = ?
#             WHERE name = ?
#         ''', (price, expiry_date, new_quantity, name))
#     else:
#         conn.execute('''
#             INSERT INTO items (name, price, expiry_date, quantity)
#             VALUES (?, ?, ?, ?)
#         ''', (name, price, expiry_date, quantity))

#     conn.commit()
#     conn.close()

#     return redirect(url_for('index'))

# @app.route('/process_image', methods=['POST'])
# def process_image_route():
#     try:
#         data = request.json
#         image_data = data['image']
        
#         header, encoded = image_data.split(',', 1)
#         image_bytes = base64.b64decode(encoded)
#         image = np.asarray(bytearray(image_bytes), dtype=np.uint8)
#         image = cv2.imdecode(image, cv2.IMREAD_COLOR)

#         detected_items = process_image(image)

#         conn = get_db_connection()
#         for item in detected_items:
#             name = item['name']
#             price = item['price']
#             expiry_date = item['expiry_date']
#             quantity = item['quantity']

#             existing_item = conn.execute('SELECT * FROM items WHERE name = ?', (name,)).fetchone()

#             if existing_item:
#                 new_quantity = existing_item['quantity'] + quantity
#                 conn.execute('''
#                     UPDATE items
#                     SET price = ?, expiry_date = ?, quantity = ?
#                     WHERE name = ?
#                 ''', (price, expiry_date, new_quantity, name))
#             else:
#                 conn.execute('''
#                     INSERT INTO items (name, price, expiry_date, quantity)
#                     VALUES (?, ?, ?, ?)
#                 ''', (name, price, expiry_date, quantity))

#         conn.commit()
#         conn.close()

#         return '', 204
#     except Exception as e:
#         return jsonify({"status": "error", "message": str(e)}), 500

# @app.route('/order_complete')
# def order_complete():
#     return render_template('result.html')

# @app.route('/reset_db', methods=['POST'])
# def reset_db():
#     try:
#         conn = get_db_connection()
#         conn.execute('DELETE FROM items')  # 모든 항목 삭제
#         conn.commit()
#         conn.close()
#         return jsonify({"status": "success", "message": "Database reset successfully"}), 200
#     except Exception as e:
#         return jsonify({"status": "error", "message": str(e)}), 500

# @app.route('/get_items')
# def get_items():
#     conn = get_db_connection()
#     items = conn.execute('SELECT * FROM items').fetchall()
#     conn.close()
#     return jsonify([dict(item) for item in items])

# def update_db_periodically():
#     while True:
#         time.sleep(0.5)  # 0.5초마다 데이터베이스 업데이트
#         conn = get_db_connection()
#         # 예시로 데이터베이스를 업데이트하는 로직
#         # 여기에 실제 업데이트 로직을 추가하세요
#         conn.close()

# if __name__ == "__main__":
#     threading.Thread(target=update_db_periodically, daemon=True).start()
#     app.run(debug=True)


import cv2
import numpy as np
from flask import Flask, request, render_template, redirect, url_for, jsonify
import sqlite3
from ultralytics import YOLO
import base64
import threading
import time
import subprocess
import logging  # 로그를 사용하기 위한 모듈

app = Flask(__name__)

# YOLOv8n 모델 로드
model = YOLO('best.pt')

# 로거 설정
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def get_db_connection():
    conn = sqlite3.connect('database.db')
    conn.row_factory = sqlite3.Row
    return conn

def process_image(image):
    results = model(image)
    detected_items = []

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            conf = box.conf[0]
            if conf > 0.5:
                class_id = int(box.cls[0].item())
                detected_item_str = result.names[class_id]
                print(f"Detected item string: {detected_item_str}")
                logging.info(f"Detected item string: {detected_item_str}")

                parts = detected_item_str.split('-')
                print(f"Parts after split: {parts}")
                logging.info(f"Parts after split: {parts}")

                if len(parts) == 5:
                    name = parts[0]
                    price = parts[1]
                    year = parts[2]
                    month = parts[3].zfill(2)
                    day = parts[4].zfill(2)
                    expiry_date = f"{year}-{month}-{day}"
                    detected_items.append({
                        'name': name,
                        'price': int(price),
                        'expiry_date': expiry_date,
                        'quantity': 1
                    })
                else:
                    error_message = f"Item data is not in the expected format: {detected_item_str}. Error: Unexpected number of parts in the item data"
                    print(error_message)
                    logging.error(error_message)

    return detected_items

def call_ros2_service():
    """ Call the ROS2 service to signal the robot to move. """
    try:
        print("ROS2 서비스 호출을 시작합니다.")
        logging.info("ROS2 서비스 호출을 시작합니다.")
        
        result = subprocess.run(
            ['ros2', 'service', 'call', '/start_parking', 'std_srvs/srv/Empty'],
            capture_output=True, 
            text=True
        )
        
        print(f"ROS2 서비스 응답: {result.stdout}")
        logging.info(f"ROS2 서비스 응답: {result.stdout}")
        
        return result.stdout
    except Exception as e:
        error_message = f"ROS2 서비스 호출 중 오류 발생: {e}"
        print(error_message)
        logging.error(error_message)
        return str(e)

@app.route('/')
def index():
    conn = get_db_connection()
    items = conn.execute('SELECT * FROM items').fetchall()
    conn.close()
    return render_template('index.html', items=items)

@app.route('/update', methods=['POST'])
def update_item():
    name = request.form['name']
    price = float(request.form['price'])
    expiry_date = request.form['expiry_date']
    quantity = int(request.form['quantity'])

    conn = get_db_connection()
    
    existing_item = conn.execute('SELECT * FROM items WHERE name = ?', (name,)).fetchone()

    if existing_item:
        new_quantity = existing_item['quantity'] + quantity
        conn.execute('''
            UPDATE items
            SET price = ?, expiry_date = ?, quantity = ?
            WHERE name = ?
        ''', (price, expiry_date, new_quantity, name))
    else:
        conn.execute('''
            INSERT INTO items (name, price, expiry_date, quantity)
            VALUES (?, ?, ?, ?)
        ''', (name, price, expiry_date, quantity))

    conn.commit()
    conn.close()

    return redirect(url_for('index'))

@app.route('/process_image', methods=['POST'])
def process_image_route():
    try:
        data = request.json
        image_data = data['image']
        
        header, encoded = image_data.split(',', 1)
        image_bytes = base64.b64decode(encoded)
        image = np.asarray(bytearray(image_bytes), dtype=np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)

        detected_items = process_image(image)

        conn = get_db_connection()
        for item in detected_items:
            name = item['name']
            price = item['price']
            expiry_date = item['expiry_date']
            quantity = item['quantity']

            existing_item = conn.execute('SELECT * FROM items WHERE name = ?', (name,)).fetchone()

            if existing_item:
                new_quantity = existing_item['quantity'] + quantity
                conn.execute('''
                    UPDATE items
                    SET price = ?, expiry_date = ?, quantity = ?
                    WHERE name = ?
                ''', (price, expiry_date, new_quantity, name))
            else:
                conn.execute('''
                    INSERT INTO items (name, price, expiry_date, quantity)
                    VALUES (?, ?, ?, ?)
                ''', (name, price, expiry_date, quantity))

        conn.commit()
        conn.close()

        return '', 204
    except Exception as e:
        logging.error(f"Image processing error: {str(e)}")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/order_complete')
def order_complete():
    print("결제 완료 버튼이 눌렸습니다. DB 초기화 및 ROS2 서비스 호출을 시작합니다.")
    logging.info("결제 완료 버튼이 눌렸습니다. DB 초기화 및 ROS2 서비스 호출을 시작합니다.")
    
    # Call the ROS2 service
    service_response = call_ros2_service()
    
    # Render the result page with the service response
    return render_template('result.html', service_response=service_response)

@app.route('/reset_db', methods=['POST'])
def reset_db():
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM items')  # 모든 항목 삭제
        conn.commit()
        conn.close()
        
        success_message = "데이터베이스가 성공적으로 초기화되었습니다."
        print(success_message)
        logging.info(success_message)
        
        return jsonify({"status": "success", "message": "Database reset successfully"}), 200
    except Exception as e:
        error_message = f"DB 초기화 중 오류 발생: {e}"
        print(error_message)
        logging.error(error_message)
        
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/get_items')
def get_items():
    conn = get_db_connection()
    items = conn.execute('SELECT * FROM items').fetchall()
    conn.close()
    return jsonify([dict(item) for item in items])

def update_db_periodically():
    while True:
        time.sleep(0.5)  # 0.5초마다 데이터베이스 업데이트
        conn = get_db_connection()
        # 예시로 데이터베이스를 업데이트하는 로직
        conn.close()

if __name__ == "__main__":
    threading.Thread(target=update_db_periodically, daemon=True).start()
    app.run(debug=True)
