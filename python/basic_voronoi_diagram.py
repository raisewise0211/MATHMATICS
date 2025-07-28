import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi

# 1. 점 집합 정의 (site)
points = np.array([
    [20, 20],
    [80, 20],
    [50, 50],
    [20, 80],
    [80, 80]
])

# 2. Voronoi 다이어그램 생성
vor = Voronoi(points)

# 3. 시각화
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111)

# 점 표시
ax.plot(points[:,0], points[:,1], 'ko', label='Sites')

# Voronoi Diagram 수동으로 그리기
for simplex in vor.ridge_vertices:
    if simplex[0] >= 0 and simplex[1] >= 0:  # 유한한 선분만
        p1 = vor.vertices[simplex[0]]
        p2 = vor.vertices[simplex[1]]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-', linewidth=2)

# 축 및 범위 설정
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_aspect('equal')
ax.set_title("Basic Voronoi Diagram")
plt.legend()
plt.grid(True)
plt.show()
