# mathmatics visualization

여러 수학들을 시각화 해서 이해하기 쉽도록 만든 것들

## Quaternion

+ QuaternionNorm(쿼터니언 정규화)

    + Q_raw = [q_x, q_y, q_z, q_w]가 주어질 때, 쿼터니언의 크기를 1로 정규화 하기 위해 쿼터니언의 norm을 계산하고 각 요소를 norm으로 나눈다.

$$ Q = \sqrt{(q_x^2+q_y^2+q_z^2+q_w^2)} $$

$$ \quad q_x' = \frac{q_x}{\|Q\|}, \quad q_y' = \frac{q_y}{\|Q\|}, \quad q_z' = \frac{q_z}{\|Q\|}, \quad q_w' = \frac{q_w}{\|Q\|} \ $$

+ Quaternion2EulerXYZ

    + 쿼터니언을 오일러 각으로 변환, 정규화된 쿼터니언(q_x,q_y,q_z,q_w), arctan2(y,x)는 두 점 사이의 상대좌표를 받아 절대각을 나타냄

    $$ t_x = arctan2(2(q_w*q_x - q_y*q_z), q_w^2-q_x^2-q_y^2+q_z^2) $$

    $$ t_y = arcsin(2(q_w*q_y+ q_x*q_z)) $$

    $$ t_z = arctan2(2(q_w*q_z - q_x*q_y), q_w^2+q_x^2-q_y^2-q_z^2) $$

+ EulerXYZ2Quaternion

    + 오일러 각을 쿼터니언으로 변환, 오일러(t_x,t_y,t_z)

    $$ s_x = sin(t_x / 2), c_x = cos(t_x / 2) $$

    $$ s_y = sin(t_y / 2), c_y = cos(t_y / 2) $$

    $$ s_z = sin(t_x / 2), c_z = cos(t_z / 2) $$

    $$ q_x = s_x*c_y*c_z + c_x*s_y*s_z $$

    $$ q_y = -s_x*c_y*s_z + c_x*s_y*c_z $$

    $$ q_z = s_x*s_y*c_z + c_x*c_y*s_z $$

    $$ q_w = -s_x*s_y*s_z + c_x*c_y*c_z $$
