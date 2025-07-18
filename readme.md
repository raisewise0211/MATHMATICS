# Mathematics Visualization

여러 수학 이론들을 시각화해서 이해하기 쉽도록 만든 프로젝트입니다.  
C++와 Python으로 구현된 수학 알고리즘들과 시각화 그래프를 포함합니다.

---

## 📊 Contents

- [Quaternion](#quaternion)
- [Kinematics](#kinematics)
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

### Forward Kinematics

로봇 매니퓰레이터의 관절 각도로부터 엔드 이펙터의 위치를 계산합니다.

$$ T_i = \begin{bmatrix} 
R_i & p_i \\
0 & 1
\end{bmatrix} $$

$$ T_{total} = T_1 \cdot T_2 \cdot ... \cdot T_n $$

### Inverse Kinematics

엔드 이펙터의 목표 위치로부터 관절 각도를 계산합니다.

$$ \theta = J^{-1} \cdot \Delta x $$

여기서 J는 자코비안 행렬입니다.

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
```

### Python Implementation

```python
import numpy as np
import matplotlib.pyplot as plt

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
python quaternion.py
python kinematics.py
python sigmoid.py
python tanh_graph.py
```

---

## 📚 References

- [Quaternion Mathematics](https://en.wikipedia.org/wiki/Quaternion)
- [Robot Kinematics](https://en.wikipedia.org/wiki/Robot_kinematics)
- [Activation Functions](https://en.wikipedia.org/wiki/Activation_function)

---

**Happy Mathematics! 🧮**
