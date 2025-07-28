# Mathematics Visualization

여러 수학 이론들을 시각화해서 이해하기 쉽도록 만든 프로젝트입니다.  
C++와 Python으로 구현된 수학 알고리즘들과 시각화 그래프를 포함합니다.

---

## 📊 Contents

- [Quaternion](#quaternion)
- [Kinematics](#kinematics)
  - [3DOF Forward Kinematics](#3dof-forward-kinematics)
  - [3DOF Inverse Kinematics](#3dof-inverse-kinematics)
  - [Differential Kinematics](#differential-kinematics)
- [Path Planning](#path-planning)
  - [Voronoi Diagram](#voronoi-diagram)
  - [Generalized Voronoi Diagram (GVD)](#generalized-voronoi-diagram-gvd)
- [Activation Functions](#activation-functions)
- [Visualization](#visualization)

---

## 🔄 Quaternion

### Quaternion Normalization (쿼터니언 정규화)

Q_raw = [q_x, q_y, q_z, q_w]가 주어질 때, 쿼터니언의 크기를 1로 정규화하기 위해 쿼터니언의 norm을 계산하고 각 요소를 norm으로 나눕니다.

$$ Q = \sqrt{(q_x^2+q_y^2+q_z^2+q_w^2)} $$

$$ \quad q_x' = \frac{q_x}{\|Q\|}, \quad q_y' = \frac{q_y}{\|Q\|}, \quad q_z' = \frac{q_z}{\|Q\|}, \quad q_w' = \frac{q_w}{\|Q\|} $$

### Quaternion to Euler XYZ

정규화된 쿼터니언(q_x,q_y,q_z,q_w)을 오일러 각으로 변환합니다. arctan2(y,x)는 두 점 사이의 상대좌표를 받아 절대각을 나타냅니다.

$$ t_x = arctan2(2(q_w*q_x - q_y*q_z), q_w^2-q_x^2-q_y^2+q_z^2) $$

$$ t_y = arcsin(2(q_w*q_y+ q_x*q_z)) $$

$$ t_z = arctan2(2(q_w*q_z - q_x*q_y), q_w^2+q_x^2-q_y^2-q_z^2) $$

### Euler XYZ to Quaternion

오일러 각(t_x,t_y,t_z)을 쿼터니언으로 변환합니다.

$$ s_x = sin(t_x / 2), c_x = cos(t_x / 2) $$

$$ s_y = sin(t_y / 2), c_y = cos(t_y / 2) $$

$$ s_z = sin(t_z / 2), c_z = cos(t_z / 2) $$

$$ q_x = s_x*c_y*c_z + c_x*s_y*s_z $$

$$ q_y = -s_x*c_y*s_z + c_x*s_y*c_z $$

$$ q_z = s_x*s_y*c_z + c_x*c_y*s_z $$

$$ q_w = -s_x*s_y*s_z + c_x*c_y*c_z $$

---

## 🤖 Kinematics

### 3DOF Forward Kinematics

3자유도 로봇 매니퓰레이터의 순기구학을 구현합니다. DH 파라미터를 사용하여 각 관절 각도로부터 엔드 이펙터의 위치를 계산합니다.

```python
# DH 파라미터 정의
dh_params = [
    [0, D1, 0, np.pi/2],   # 링크 1
    [0, D2, 0, np.pi/2],   # 링크 2  
    [0, D3, 0, 0]          # 링크 3
]

def forward_kinematics(joint_angles):
    T = np.identity(4)
    for i in range(3):
        theta = dh_params[i][0] + joint_angles[i]
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]
        T_i = dh_transform(theta, d, a, alpha)
        T = np.dot(T, T_i)
    return T
```

### 3DOF Inverse Kinematics

3자유도 로봇의 역기구학을 구현합니다. 엔드 이펙터의 목표 위치로부터 관절 각도를 계산합니다.

```python
def inverse_kinematics(x, y, z):
    joint1 = np.arctan2(y, x)
    r = np.sqrt(x**2 + y**2)
    h = z - D1
    L1 = D2
    L2 = D3
    D = (r**2 + h**2 - L1**2 - L2**2) / (2 * L1 * L2)
    joint3 = np.arctan2(np.sqrt(1 - D**2), D)
    phi = np.arctan2(h, r)
    psi = np.arctan2(L2 * np.sin(joint3), L1 + L2 * np.cos(joint3))
    joint2 = phi - psi
    return [joint1, joint2, joint3]
```

### Differential Kinematics

차동 구동 로봇의 운동학을 구현합니다. 선속도와 각속도로부터 좌우 바퀴의 RPM을 계산합니다.

```cpp
// 선속도/각속도 → 좌우 바퀴 RPM 변환
std::pair<motor::left,motor::right> Kinematics::VWtoRPM(geometry_msgs::Twist vel)
{
    std::pair<motor::left,motor::right> rpm;
    rpm.first = ((60/(2*M_PI*wheel_radius)) * 
                 (vel.linear.x - (wheel_distance/2)*vel.angular.z) * gear_ratio);
    rpm.second = ((60/(2*M_PI*wheel_radius)) * 
                  (vel.linear.x + (wheel_distance/2)*vel.angular.z) * gear_ratio);
    return rpm;
}
```

---

## 🗺️ Path Planning

### Voronoi Diagram

기본 Voronoi 다이어그램을 구현합니다. 점들의 집합으로부터 각 점에 가장 가까운 영역을 구분합니다.

```python
def basic_voronoi_diagram():
    # 점들의 집합 정의
    points = np.array([[0, 0], [1, 1], [2, 0], [1, -1]])
    
    # Voronoi 다이어그램 계산
    vor = Voronoi(points)
    
    # 시각화
    fig, ax = plt.subplots()
    voronoi_plot_2d(vor, ax)
    ax.plot(points[:, 0], points[:, 1], 'ko')
    plt.show()
```

### Generalized Voronoi Diagram (GVD)

장애물이 있는 환경에서의 일반화된 Voronoi 다이어그램을 구현합니다. 장애물로부터의 거리 맵을 기반으로 GVD를 추출하고, A* 알고리즘을 사용하여 경로 계획을 수행합니다.

```python
def compute_gvd(obstacle_map):
    # 거리 변환 계산
    dist_map = distance_transform_edt(1 - obstacle_map)
    dist_blur = cv2.GaussianBlur(dist_map, (3,3), 0)
    
    # GVD 추출 (local maximum 검출)
    gvd = np.zeros_like(obstacle_map)
    for y in range(1, dist_blur.shape[0] - 1):
        for x in range(1, dist_blur.shape[1] - 1):
            center = dist_blur[y, x]
            neighbors = dist_blur[y-1:y+2, x-1:x+2]
            if center > 0 and np.count_nonzero(center >= neighbors) >= 6:
                gvd[y, x] = 255
    
    return gvd.astype(np.uint8)

def astar(start, goal, gvd_map):
    # A* 알고리즘으로 GVD 상에서 경로 탐색
    h, w = gvd_map.shape
    open_set = []
    came_from = {}
    g_score = {start: 0}
    f_score = {start: np.linalg.norm(np.array(start) - np.array(goal))}
    heappush(open_set, (f_score[start], start))
    
    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            # 경로 재구성
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        
        # 8방향 탐색
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < h and 0 <= neighbor[1] < w:
                if gvd_map[neighbor] == 0:
                    continue
                tentative_g = g_score[current] + np.linalg.norm(np.array(neighbor) - np.array(current))
                if tentative_g < g_score.get(neighbor, np.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + np.linalg.norm(np.array(neighbor) - np.array(goal))
                    heappush(open_set, (f_score[neighbor], neighbor))
    
    return None
```

---

## 🧠 Activation Functions

### Sigmoid Function

$$ \sigma(x) = \frac{1}{1 + e^{-x}} $$

```python
def sigmoid(x):
    return 1 / (1 + np.exp(-x))
```

### Hyperbolic Tangent (Tanh)

$$ \tanh(x) = \frac{e^x - e^{-x}}{e^x + e^{-x}} $$

```python
def tanh(x):
    return np.tanh(x)
```

---

## 📈 Visualization

### Sigmoid Function Graph

```python
import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(-10, 10, 100)
y = 1 / (1 + np.exp(-x))

plt.plot(x, y)
plt.title('Sigmoid Function')
plt.xlabel('x')
plt.ylabel('σ(x)')
plt.grid(True)
plt.show()
```

### Tanh Function Graph

```python
x = np.linspace(-5, 5, 100)
y = np.tanh(x)

plt.plot(x, y)
plt.title('Hyperbolic Tangent Function')
plt.xlabel('x')
plt.ylabel('tanh(x)')
plt.grid(True)
plt.show()
```

### 3D Robot Arm Visualization

```python
def visualize_robot_arm(joint_angles):
    positions = get_joint_positions(joint_angles)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    xs = positions[:, 0]
    ys = positions[:, 1]
    zs = positions[:, 2]
    
    ax.plot(xs, ys, zs, '-o', label='Robot Arm', markersize=8)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3DOF Robot Arm')
    plt.show()
```

---

## 🛠️ Implementation

### C++ Implementation

```cpp
// Quaternion class
class Quaternion {
private:
    double x, y, z, w;
    
public:
    Quaternion(double x, double y, double z, double w) 
        : x(x), y(y), z(z), w(w) {}
    
    void normalize() {
        double norm = sqrt(x*x + y*y + z*z + w*w);
        x /= norm; y /= norm; z /= norm; w /= norm;
    }
    
    // Quaternion to Euler conversion
    void toEuler(double& roll, double& pitch, double& yaw) {
        roll = atan2(2*(w*x - y*z), w*w - x*x - y*y + z*z);
        pitch = asin(2*(w*y + x*z));
        yaw = atan2(2*(w*z - x*y), w*w + x*x - y*y - z*z);
    }
};

// Differential Kinematics
class Kinematics {
private:
    double wheel_radius, wheel_distance, gear_ratio;
    
public:
    std::pair<motor::left,motor::right> VWtoRPM(geometry_msgs::Twist vel) {
        std::pair<motor::left,motor::right> rpm;
        rpm.first = ((60/(2*M_PI*wheel_radius)) * 
                     (vel.linear.x - (wheel_distance/2)*vel.angular.z) * gear_ratio);
        rpm.second = ((60/(2*M_PI*wheel_radius)) * 
                      (vel.linear.x + (wheel_distance/2)*vel.angular.z) * gear_ratio);
        return rpm;
    }
};
```

### Python Implementation

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from heapq import heappush, heappop

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w
    
    def normalize(self):
        norm = np.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        self.x /= norm; self.y /= norm; self.z /= norm; self.w /= norm
    
    def to_euler(self):
        roll = np.arctan2(2*(self.w*self.x - self.y*self.z), 
                          self.w**2 - self.x**2 - self.y**2 + self.z**2)
        pitch = np.arcsin(2*(self.w*self.y + self.x*self.z))
        yaw = np.arctan2(2*(self.w*self.z - self.x*self.y), 
                         self.w**2 + self.x**2 - self.y**2 - self.z**2)
        return roll, pitch, yaw

class GVDPlanner:
    def __init__(self):
        pass
    
    def compute_gvd(self, obstacle_map):
        dist_map = distance_transform_edt(1 - obstacle_map)
        dist_blur = cv2.GaussianBlur(dist_map, (3,3), 0)
        
        gvd = np.zeros_like(obstacle_map)
        for y in range(1, dist_blur.shape[0] - 1):
            for x in range(1, dist_blur.shape[1] - 1):
                center = dist_blur[y, x]
                neighbors = dist_blur[y-1:y+2, x-1:x+2]
                if center > 0 and np.count_nonzero(center >= neighbors) >= 6:
                    gvd[y, x] = 255
        
        return gvd.astype(np.uint8)
    
    def astar(self, start, goal, gvd_map):
        # A* 알고리즘 구현
        h, w = gvd_map.shape
        open_set = []
        came_from = {}
        g_score = {start: 0}
        f_score = {start: np.linalg.norm(np.array(start) - np.array(goal))}
        heappush(open_set, (f_score[start], start))
        
        while open_set:
            _, current = heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < h and 0 <= neighbor[1] < w:
                    if gvd_map[neighbor] == 0:
                        continue
                    tentative_g = g_score[current] + np.linalg.norm(np.array(neighbor) - np.array(current))
                    if tentative_g < g_score.get(neighbor, np.inf):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + np.linalg.norm(np.array(neighbor) - np.array(goal))
                        heappush(open_set, (f_score[neighbor], neighbor))
        
        return None
```

---

## 🚀 Usage

### C++ 실행
```bash
cd c++
mkdir build && cd build
cmake ..
make
./mathmatics_demo
```

### Python 실행
```bash
cd python

# 기본 수학 함수들
python quaternion.py
python sigmoid.py
python tanh_graph.py

# 기구학
python 3dof_forwardkinematic.py
python 3dof_inverse_kinematic.py
python kinematics.py

# 경로 계획
python basic_voronoi_diagram.py
python generalized_voronoi_diagram.py
python gvd_path_planning.py
```

---

## 📚 References

- [Quaternion Mathematics](https://en.wikipedia.org/wiki/Quaternion)
- [Robot Kinematics](https://en.wikipedia.org/wiki/Robot_kinematics)
- [Voronoi Diagram](https://en.wikipedia.org/wiki/Voronoi_diagram)
- [Generalized Voronoi Diagram](https://en.wikipedia.org/wiki/Generalized_Voronoi_diagram)
- [A* Search Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Activation Functions](https://en.wikipedia.org/wiki/Activation_function)

---

**Happy Mathematics! 🧮**
