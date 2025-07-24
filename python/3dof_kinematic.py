import numpy as np

# 3자유도 로봇의 DH 파라미터 (예시값, 실제 로봇에 맞게 수정 필요)
D1 = 1.0
D2 = 1.0
D3 = 1.0

# [theta, d, a, alpha]
dh_params = [
    [0,     D1,      0,      np.pi/2 ],   # 링크 1
    [0,     D2,      0,      -np.pi/2],   # 링크 2
    [0,     D3,      0,      0      ]    # 링크 3
]

def dh_transform(theta, d, a, alpha):
    """
    개별 DH 파라미터를 사용하여 변환 행렬을 계산합니다.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics(joint_angles):
    """
    각 관절 각도를 입력 받아 엔드 이펙터의 변환 행렬을 계산합니다.
    """
    T = np.identity(4)
    for i in range(3):
        theta = dh_params[i][0] + joint_angles[i]
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]
        T_i = dh_transform(theta, d, a, alpha)
        T = np.dot(T, T_i)
    return T

# ====== 사용자 입력 추가 ======
print("각 관절의 각도를 도(degree) 단위로 입력하세요.")
joint_angles = []
for i in range(3):
    angle = float(input(f"{i+1}번 관절 각도: "))
    joint_angles.append(np.deg2rad(angle))

# 엔드 이펙터의 변환 행렬 계산
T_end = forward_kinematics(joint_angles)
print(joint_angles)
# 결과 출력
print("엔드 이펙터의 위치와 자세:")
print(T_end)

# 엔드 이펙터의 위치 추출
position = T_end[:3, 3]
print("\n엔드 이펙터의 위치 (x, y, z):")
print(position)

# ====== 시각화 코드 (joint_angles 변수 재사용) ======
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_joint_positions(joint_angles):
    """
    각 관절 위치(원점~엔드이펙터)를 3D 좌표로 반환합니다.
    """
    positions = [np.array([0, 0, 0, 1])]  # base(원점)
    T = np.identity(4)
    for i in range(3):
        theta = dh_params[i][0] + joint_angles[i]
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]
        T_i = dh_transform(theta, d, a, alpha)
        T = np.dot(T, T_i)
        pos = T @ np.array([0, 0, 0, 1])
        positions.append(pos)
    return np.array(positions)

positions = get_joint_positions(joint_angles)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 관절 위치 좌표 추출
xs = positions[:, 0]
ys = positions[:, 1]
zs = positions[:, 2]

# 로봇 팔 그리기
ax.plot(xs, ys, zs, '-o', label='Robot Arm', markersize=8)

# 각 조인트에 텍스트 추가
joint_labels = ["joint1", "joint2", "joint3", "end_effector"]
for i, label in enumerate(joint_labels):
    ax.text(xs[i], ys[i], zs[i], label, fontsize=12, color='blue')

# 엔드이펙터 강조
ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=100, label='End Effector')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3DOF robot arm visualization')
ax.legend()
ax.grid(True)
plt.show() 