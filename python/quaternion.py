import math

def QuaternionNorm(Q_raw):
    qx_temp, qy_temp, qz_temp, qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp * qx_temp + qy_temp * qy_temp + qz_temp * qz_temp + qw_temp * qw_temp)
    qx_ = qx_temp / qnorm
    qy_ = qy_temp / qnorm
    qz_ = qz_temp / qnorm
    qw_ = qw_temp / qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_

def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_, qy_, qz_, qw_ = Q_normed

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_, ty_, tz_]
    return EulerXYZ_

def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_

def main():
    print("Choose a function to run:")
    print("1. Convert Euler angles to Quaternion")
    print("2. Convert Quaternion to Euler angles")
    
    choice = input("Enter your choice (1 or 2): ")
    
    if choice == "1":
        EulerXYZ_ = list(map(float, input("Enter three values for Euler angles (tx, ty, tz): ").split()))
        result = EulerXYZ2Quaternion(EulerXYZ_)
        print("Quaternion:", result)
    
    elif choice == "2":
        Q_raw = list(map(float, input("Enter four values for the quaternion (qx, qy, qz, qw): ").split()))
        result = Quaternion2EulerXYZ(Q_raw)
        print("Euler Angles (XYZ):", result)
    
    else:
        print("Invalid choice. Please enter 1 or 2.")

if __name__ == "__main__":
    main()
