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

### 3DOF Inverse Kinematics

3자유도 로봇의 역기구학을 구현합니다. 엔드 이펙터의 목표 위치로부터 관절 각도를 계산합니다.

### Differential Kinematics

차동 구동 로봇의 운동학을 구현합니다. 선속도와 각속도로부터 좌우 바퀴의 RPM을 계산합니다.

---

## 🗺️ Path Planning

### Voronoi Diagram

기본 Voronoi 다이어그램을 구현합니다. 점들의 집합으로부터 각 점에 가장 가까운 영역을 구분합니다.

### Generalized Voronoi Diagram (GVD)

장애물이 있는 환경에서의 일반화된 Voronoi 다이어그램을 구현합니다. 장애물로부터의 거리 맵을 기반으로 GVD를 추출하고, A* 알고리즘을 사용하여 경로 계획을 수행합니다.

---

## 🧠 Activation Functions

### Sigmoid Function

$$ \sigma(x) = \frac{1}{1 + e^{-x}} $$

### Hyperbolic Tangent (Tanh)

$$ \tanh(x) = \frac{e^x - e^{-x}}{e^x + e^{-x}} $$

---

## 📈 Visualization

### Sigmoid Function Graph

시그모이드 함수의 그래프를 시각화합니다.

### Tanh Function Graph

쌍곡선 탄젠트 함수의 그래프를 시각화합니다.

### 3D Robot Arm Visualization

3자유도 로봇 팔의 3D 시각화를 제공합니다.

---

## 🛠️ Implementation

### C++ Implementation

- Quaternion 클래스: 쿼터니언 연산 및 오일러 각 변환
- Differential Kinematics 클래스: 차동 구동 로봇의 운동학 계산

### Python Implementation

- Quaternion 클래스: 쿼터니언 정규화 및 변환
- GVDPlanner 클래스: 일반화된 Voronoi 다이어그램 및 A* 경로 계획
- 3DOF Kinematics: 순기구학 및 역기구학 계산

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
