import sqlite3
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm
# 데이터베이스 열기
conn = sqlite3.connect('/home/sineunji/ros2_ws/src/storagy_exhibition_ui/database/graph')  # 'example.db'는 열고자 하는 SQLite 데이터베이스 파일입니다.
cursor = conn.cursor()

#월드 좌표 
# 태그 ID에 대한 월드 좌표 설정 (Map 좌표계 기준)
tag_coordinates = {
                0: (4.987993240, -1.04322886),
                1: (4.546353340148926, 1.1332958936691284),
                2: (3.42607712, -0.445754587),
                3: (1.4073152542, 0.645906567),
                4: (3.2566099166870117, -2.4146056175231934),
                5: (3.002598, -2.4304530),
                6: (2.8743224143, 1.7236061096),
                7: (4.74539375, 0.558673858),
                8: (5.55115229, 0.275051201),
                9: (4.7196235, -2.5375328),
                10: (2.303166627, 1.28135883808),
                11: (2.0105538368, -2.5801603794),
                12: (3.857791, -2.4696977),
                13: (4.226512908935547, -2.4479730129241943),
                15: (5.529539108276367, -0.2425769716501236),
                16: (2.084618091583252, 1.154811143875122),
                17: (2.3057069778442383, 1.5482909679412842),
                18: (1.478023886680603, 0.9653726816177368),
                19: (1.7643160820007324, 1.1299530267715454),
                20: (4.258351802825928, 1.7059189081192017),
                21: (3.8528831005096436, 1.7191468477249146),
                22: (3.4360427856445312, 1.7011438608169556),
                23: (5.220964431762695, 0.5418509244918823),
                24: (4.613394260406494, 0.8273558020591736),
                25: (4.511187553405762, 1.3130862712860107) }



#좌표 변환 
def transform_coord():
    #초기화
    coordinates = []
    # 'example_table'에서 모든 데이터를 선택하는 쿼리
    cursor.execute('SELECT time,"index" FROM tracking_graph')
    # 데이터베이스 결과 가져오기
    results = cursor.fetchall()
    # 고객 수 row[0] = 시간대 (text) row [1] = 좌표 태그 숫자 리스트 (text) 
    for row in results :   
        # row[1]을 문자열로 처리하여 공백 기준으로 분리 후 정수 변환
        index_str = row[1].strip("[]' ")
        index_list = list(map(int, index_str.split(',')))
        coordinates = [tag_coordinates.get(i) for i in index_list if i in tag_coordinates]
        
        # 좌표 분리
        x_coords = [x for x, y in coordinates]     
        y_coords = [y for x, y in coordinates]

        # 경로 그리기
        plt.plot(x_coords, y_coords, marker='o',linewidth= 4 ,label=f'Path {row[0]}') 
    
    # 그래프 설정
    plt.title('Tag')
    plt.xlabel('X axis')
    plt.ylabel('Y axis')
    plt.legend()  # 범례 추가
    plt.grid(True)  # 그리드 추가
    plt.show()        


# 좌표 변환 및 시간대별 히스토그램 그리기
def plot_stacked_histogram():
    # 'tracking_graph'에서 모든 데이터를 선택하는 쿼리
    cursor.execute('SELECT time, "index" FROM tracking_graph')
    # 데이터베이스 결과 가져오기
    results = cursor.fetchall()

    # 시간대별 인덱스 데이터 저장할 리스트 초기화
    all_indices = []
    labels = []

    for row in results:
        time = row[0]  # 시간대
        index_str = row[1].strip("[]' ")
        index_list = list(map(int, index_str.split(',')))  # 쉼표 기준으로 분리 후 정수 리스트로 변환

        all_indices.append(index_list)
        labels.append(time)

    # 히스토그램 그리기 (stacked)
    plt.hist(all_indices, bins=range(min(min(all_indices)), max(max(all_indices)) + 2), 
             alpha=0.7, edgecolor='black', label=labels, stacked=True)
    
    plt.title('Stacked Histogram of Index Data')
    plt.xlabel('Index Value')
    plt.ylabel('Frequency')
    plt.xticks(range(min(min(all_indices)), max(max(all_indices)) + 1))  # x축 눈금 간격을 1로 설정
    plt.legend()  # 범례 추가
    plt.grid(True)
    plt.show()

# 좌표 변환 및 시간대별 히스토그램 그리기
def plot_histogram_with_centered_y():
    # 'tracking_graph'에서 모든 데이터를 선택하는 쿼리
    cursor.execute('SELECT time, "index" FROM tracking_graph')
    # 데이터베이스 결과 가져오기
    results = cursor.fetchall()

    # 시간대별 인덱스 데이터 저장할 리스트 초기화
    all_indices = []
    labels = []

    for row in results:
        time = row[0]  # 시간대
        index_str = row[1].strip("[]' ")
        index_list = list(map(int, index_str.split(',')))  # 쉼표 기준으로 분리 후 정수 리스트로 변환

        all_indices.append(index_list)
        labels.append(time)

    # 모든 데이터를 하나의 리스트로 결합하여 빈도 계산
    combined_indices = [index for sublist in all_indices for index in sublist]
    counts, bins = np.histogram(combined_indices, bins=range(min(combined_indices), max(combined_indices) + 2))
    
    # y값 정렬: 빈도수에 따라 정렬된 인덱스 값들로 새로운 순서 생성
    sorted_indices = sorted(zip(bins[:-1], counts), key=lambda x: x[1], reverse=True)
    sorted_indices_dict = {x[0]: rank for rank, x in enumerate(sorted_indices)}

    # 재정렬된 y값에 따라 인덱스를 재배열
    rearranged_all_indices = [[sorted_indices_dict[i] for i in sublist] for sublist in all_indices]

    # 히스토그램 그리기 (stacked)
    plt.hist(rearranged_all_indices, bins=range(max(max(rearranged_all_indices)) + 2), 
             alpha=0.7, edgecolor='black', label=labels, stacked=True)
    
    plt.title('Customer Movement Analysis Based on Tag ID Tracking')
    plt.xlabel('X: Apriltag_ID')
    plt.ylabel('Y: Num of Apriltag detection')
    plt.xticks([sorted_indices_dict[x] for x in bins[:-1]], bins[:-1])  # x축에 원래 인덱스 값 표시
    plt.legend()  # 범례 추가
    plt.grid(True)
    plt.show()

# 히스토그램 그리기 함수 실행
plot_histogram_with_centered_y()
