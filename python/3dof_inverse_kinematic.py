import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import time

# DH 파라미터 (3dof_kinematic.py와 동일하게 맞춤)
D1 = 1.0
D2 = 1.0
D3 = 1.0

# 순기구학 함수 (조인트 각도 → 각 조인트 위치)
def forward_kinematics(joint_angles):
    dh_params = [
        [0,     D1,      0,      np.pi/2 ],   # 링크 1
        [0,     D2,      0,      -np.pi/2],   # 링크 2
        [0,     D3,      0,      0      ]    # 링크 3
    ]
    positions = [np.array([0, 0, 0, 1])]  # base(원점)
    T = np.identity(4)
    for i in range(3):
        theta = dh_params[i][0] + joint_angles[i]
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]
        T_i = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
        T = np.dot(T, T_i)
        pos = T @ np.array([0, 0, 0, 1])
        positions.append(pos)
    return np.array(positions)

def inverse_kinematics(x, y, z):
    joint1 = np.arctan2(y, x)
    r = np.sqrt(x**2 + y**2)
    h = z - D1
    L1 = D2
    L2 = D3
    D = (r**2 + h**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if np.abs(D) > 1.0:
        raise ValueError("해당 위치는 로봇팔이 도달할 수 없습니다.")
    joint3 = np.arctan2(np.sqrt(1 - D**2), D)
    phi = np.arctan2(h, r)
    psi = np.arctan2(L2 * np.sin(joint3), L1 + L2 * np.cos(joint3))
    joint2 = phi - psi
    return [joint1, joint2, joint3]

# 랜덤 목표 위치 생성 (로봇팔이 도달 가능한 범위 내)
def random_target():
    # 최대 도달 반경: D2 + D3
    r_max = D2 + D3 - 0.1
    r = np.random.uniform(0.2, r_max)
    theta = np.random.uniform(-np.pi, np.pi)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.random.uniform(D1 - 0.5, D1 + D2 + D3 - 0.2)
    return x, y, z

# 시각화 세팅
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 3)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3DOF robot arm inverse kinematics')

line, = ax.plot([], [], [], '-o', markersize=8, label='Robot Arm')
target_point = ax.scatter([], [], [], color='red', s=80, label='Target')

joint_labels = ["joint1", "joint2", "joint3", "end_effector"]
text_objs = [ax.text(0,0,0,"") for _ in range(4)]

# 애니메이션 프레임 함수
def update(frame):
    ax.collections.clear()  # target_point 초기화
    x, y, z = random_target()
    try:
        joints = inverse_kinematics(x, y, z)
        positions = forward_kinematics(joints)
        xs = positions[:, 0]
        ys = positions[:, 1]
        zs = positions[:, 2]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        ax.scatter([x], [y], [z], color='red', s=80, label='Target')
        # 조인트 텍스트
        for i, label in enumerate(joint_labels):
            text_objs[i].set_position((xs[i], ys[i]))
            text_objs[i].set_3d_properties(zs[i])
            text_objs[i].set_text(label)
            text_objs[i].set_color('blue')
            text_objs[i].set_fontsize(8)
        return line, *text_objs
    except ValueError:
        # 도달 불가한 위치면 skip
        return line, *text_objs

ani = FuncAnimation(fig, update, frames=100, interval=1000, blit=False)
ax.legend()
plt.show() 