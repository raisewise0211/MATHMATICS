import numpy as np

D1 = 0.2755
D2 = 0.4100
D3 = 0.2073
D4 = 0.1600
e2 = 0.0098

# 각 관절의 DH 파라미터를 정의합니다.
# [theta, d, a, alpha]
dh_params = [
    [0,     D1,      0,      np.pi/2 ],   # 링크 1
    [0,     0,      D2,      np.pi   ],         # 링크 2
    [0,     -e2,      0,      np.pi/2 ],         # 링크 3
    [0,     (D3+D4),      0,      np.pi   ]          # 링크 4
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
    for i in range(4):
        theta = dh_params[i][0] + joint_angles[i]
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]
        T_i = dh_transform(theta, d, a, alpha)
        T = np.dot(T, T_i)
    return T

# 각 관절의 회전 각도 (라디안 단위)
joint_angles = [np.deg2rad(275), np.deg2rad(175), np.deg2rad(79), np.deg2rad(179)]

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
