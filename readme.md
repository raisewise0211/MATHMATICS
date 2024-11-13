#mathmatics visualization

여러 수학들을 시각화 해서 이해하기 쉽도록 만든 것들

##Quaternion

+ QuaternionNorm(쿼터니언 정규화)
    +Q_raw = [q_x, q_y, q_z, q_w]가 주어질 때, 쿼터니언의 크기를 1로 정규화 하기 위해 쿼터니언의 norm을 계산하고 각 요소를 norm으로 나눈다.

Q = sqrt((q_x)^2+(q_y)^2+(q_z)^2+(q_w)^2)

q_x' = \frac{q_x}{\|Q\|}, \quad q_y' = \frac{q_y}{\|Q\|}, \quad q_z' = \frac{q_z}{\|Q\|}, \quad q_w' = \frac{q_w}{\|Q\|}

For a normalized quaternion \( Q_{\text{normed}} = [q_x', q_y', q_z', q_w'] \):

$$\( q_x' = \frac{q_x}{\|Q\|}, \quad q_y' = \frac{q_y}{\|Q\|}, \quad q_z' = \frac{q_z}{\|Q\|}, \quad q_w' = \frac{q_w}{\|Q\|} \)$$
