import numpy as np
import matplotlib.pyplot as plt

# 시그모이드 함수 정의
def sigmoid(a):
    return 1 / (1 + np.exp(-a))

# a 값 생성 (-10에서 10 사이의 400개 값)
a_values = np.linspace(-10, 10, 400)

# 각 a 값에 대한 시그모이드 값 계산
sigmoid_values = sigmoid(a_values)

# 그래프 그리기
plt.plot(a_values, sigmoid_values, label="sigmoid(a)", color='green')
plt.title("Graph of Sigmoid Function")
plt.xlabel("a")
plt.ylabel("sigmoid(a)")
plt.grid(True)
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.legend()
plt.show()
