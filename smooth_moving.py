from math import cos, sin, tan, pi, sqrt, atan


# ----------- Решение уравнения движения с ускорением -------------
def get_smooth_moving_by_speed(distance, time_total, time_cur):
    s1 = a * t**2 / 2
    s2 =


if __name__ == '__main__':
    F = 1
    E = 0.3
    Lf = 0.7
    Le = 1.2

    x = 0
    y = 0.5
    z = -1  # должно быть < 0 (вниз, под базой)

    print(f"Point: [{x}, {y}, {z}]")

    print("")
    print("Inverse kinematics:")
    res = inverse_kinematics(x, y, z, F, E, Lf, Le)
    if res is None:
        print("Solve not existing")
        exit()

    (theta_1, theta_2, theta_3) = res
    print('theta 1:', theta_1)
    print('theta 2:', theta_2)
    print('theta 3:', theta_3)
    (t1, t2, t3) = inverse_kinematics_lowperf(x, y, z, F, E, Lf, Le)
    print("LowPerf result:", t1/pi*180 % 180, t2/pi*180 % 180, t3/pi*180 % 180)

    print("")
    print("Straight kinematics:")
    res = straight_kinematics(theta_1, theta_2, theta_3, F, E, Lf, Le)
    if res is None:
        print("Solve not existing")
        exit()
    (x0, y0, z0) = res
    print('x0:', x0)
    print('y0:', y0)
    print('z0:', z0)
    (x0_1, y0_1, z0_1) = straight_kinematics_lowperf(theta_1, theta_2, theta_3, F, E, Lf, Le)
    print("LowPerf result:", x0_1, y0_1, z0_1)
