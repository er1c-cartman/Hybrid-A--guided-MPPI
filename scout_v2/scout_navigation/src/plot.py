import numpy as np
import matplotlib.pyplot as plt

# 텍스트 파일에서 행렬 불러오기 (cost map과 경로)
costMap = np.loadtxt("/home/home/paper_ws/src/occ_map.txt")
path = np.loadtxt("/home/home/paper_ws/src/path.txt")

# costMap을 이미지로 플롯 (격자 형태로 시각화)
plt.imshow(costMap, cmap='gray_r', interpolation='none')
plt.colorbar(label='Cost')

plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Cost Map')

# 각 셀에 값 표시 (원하는 경우)
for i in range(costMap.shape[0]):
    for j in range(costMap.shape[1]):
        plt.text(j, i, int(costMap[i, j]), ha='center', va='center', color='blue')

# 경로를 빨간색 선으로 표시
plt.plot(path[:, 0], path[:, 1], 'ro-', label='Path')  # 'ro-'로 경로 표시 (빨간색 원과 선)

# 경로의 점을 강조하고 싶다면 아래 코드로 표시 가능
plt.scatter(path[:, 0], path[:, 1], color='red')

plt.legend()  # 범례 추가 (Path 설명)
plt.show()
