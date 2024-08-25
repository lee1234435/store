import cv2
import numpy as np
from flask import Flask, request, render_template, redirect, url_for
import sqlite3
from ultralytics import YOLO
from pyzbar.pyzbar import decode
import base64
from io import BytesIO

app = Flask(__name__)

# YOLOv8n 모델 로드
model = YOLO('yolov8n.pt')  # YOLOv8n 모델의 사전 학습된 가중치를 로드합니다

def get_db_connection():
    conn = sqlite3.connect('database.db')
    conn.row_factory = sqlite3.Row
    return conn

def process_image(image):
    # YOLOv8n 모델을 사용하여 이미지에서 바코드를 인식합니다
    results = model(image)
    detected_barcodes = []
    
    for result in results.xyxy[0]:  # bounding box
        x1, y1, x2, y2, conf, cls = result
        if conf > 0.5:  # Confidence threshold
            # Crop and process the detected barcode area
            cropped_image = image[int(y1):int(y2), int(x1):int(x2)]
            barcodes = decode(cropped_image)
            for barcode in barcodes:
                detected_barcodes.append(barcode.data.decode('utf-8'))

    return detected_barcodes

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
    data = request.json
    image_data = data['image']
    
    # Remove the metadata from the base64 image data
    header, encoded = image_data.split(',', 1)
    image_bytes = base64.b64decode(encoded)
    image = np.asarray(bytearray(image_bytes), dtype=np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)

    barcodes = process_image(image)

    conn = get_db_connection()
    for barcode in barcodes:
        # Assuming barcode contains the product name, price, expiry_date, quantity
        # Split the barcode data into the required fields (adjust if needed)
        name, price, expiry_date, quantity = barcode.split(',')  # Update this as per your barcode format
        price = float(price)
        quantity = int(quantity)

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


@app.route('/order_complete')
def order_complete():
    return render_template('result.html')

if __name__ == "__main__":
    app.run(debug=True)