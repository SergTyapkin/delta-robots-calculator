from sympy import symbols, sqrt, atan, cos, sin, tan, pi
import math
from line_profiler import profile
import os
os.environ["LINE_PROFILE"] = "0" # Enable or disable line_profiler

# SymPy prettified
tan30 = symbols('tan30')
tan60 = symbols('tan60')
sin30 = symbols('sin30')
cos120 = -1/2
cosMinus120 = -1/2
sin120 = symbols('sqrt(3)') / 2
sinMinus120 = -symbols('sqrt(3)') / 2

# Numbered
# tan30 = tan(30 / 180 * pi)
# tan60 = tan(60 / 180 * pi)
# sin30 = sin(30 / 180 * pi)


# ----------- ПРЯМАЯ КИНЕМАТИКА -------------
# Прямая кинематика: (theta1, theta2, theta3) в градусах -> (x, y, z) рабочего органа
# theta_1 - угол привода, расположенного на оси Y
# theta_2 - угол привода, повернутого от первого против часовой стрелки на 120 градусов
# theta_3 - угол привода, повернутого от первого по часовой стрелке на 120 градусов
# F - сторона базы (верхней плиты)
# E - сторона платформы (нижней плиты)
# LF - длина плеча (верхней перемычки)
# LE - длина рычага (нижней перемычки)
@profile
def straight_kinematics(theta_1, theta_2, theta_3, F, E, LF, LE):
    t = (F - E) * tan30 / 2

    y1 = -(t + LF * cos(theta_1))
    x1 = 0
    z1 = -LF * sin(theta_1)

    y2 = (t + LF * cos(theta_2)) * sin30
    x2 = y2 * tan60
    z2 = -LF * sin(theta_2)

    y3 = (t + LF * cos(theta_3)) * sin30
    x3 = -y3 * tan60
    z3 = -LF * sin(theta_3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = x1 * x1 + y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    # x = (a1*z + b1)/dnm
    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2

    # y = (a2*z + b2)/dnm
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2

    # a*z^2 + b*z + c = 0
    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - LE * LE)

    # дискриминант
    d = b * b - 4 * a * c
    # if d < 0:
    #     return None  # несуществующая позиция

    z0 = -0.5 * (b + sqrt(d)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm
    return x0, y0, z0


# ----------- ОБРАТНАЯ КИНЕМАТИКА -------------
# вспомогательная функция, расчет угла theta1 (в плоскости YZ)
# (в градусах поворота относительно горизонтального луча по направлению от базы)
def _calcAngleYZ(x0, y0, z0, F, E, LF, LE):
    y1 = -F / 2 * tan30  # сдвигаем центр к краю верхней базы (к приводу)
    y0 -= E / 2 * tan30  # сдвигаем центр к краю нижней платформы
    # z = a + b*y
    if z0 == 0:
        return None  # несуществующая точка
    a = (x0 * x0 + y0 * y0 + z0 * z0 + LF * LF - LE * LE - y1 * y1) / (2 * z0)
    b = (y1 - y0) / z0
    # дискриминант
    d = -(a + b * y1) * (a + b * y1) + LF * (b * b * LF + LF)
    # if d < 0:
    #     return None  # несуществующая точка
    yj = (y1 - a * b - sqrt(d)) / (b * b + 1)  # выбираем внешнюю точку
    zj = a + b * yj
    # return atan(-zj / (y1 - yj)) / pi * 180 + 180  # + (180 if (yj > y1) else 0)
    return atan(-zj / (y1 - yj)) + pi  # + (180 if (yj > y1) else 0)


def rotate2Dsympy(x, y, cosDeg, sinDeg, x0=0, y0=0):
    adjusted_x = (x - x0)
    adjusted_y = (y - y0)
    qx = x0 + cosDeg * adjusted_x + sinDeg * adjusted_y
    qy = y0 + -sinDeg * adjusted_x + cosDeg * adjusted_y

    return qx, qy

def rotate2D(x, y, degrees, x0=0, y0=0):
    rad = degrees / 180 * pi
    adjusted_x = (x - x0)
    adjusted_y = (y - y0)
    cos_rad = cos(rad)
    sin_rad = sin(rad)
    qx = x0 + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = y0 + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return qx, qy

# Обратная кинематика: (x0, y0, z0) -> (theta1, theta2, theta3) в градусах
# При этом theta_1 - угол привода, расположенного на оси Y
# theta_2 - угол привода, повернутого от первого против часовой стрелки на 120 градусов
# theta_3 - угол привода, повернутого от первого по часовой стрелке на 120 градусов
# ---
# x, y, z - координаты точки для поиска углов
# F - сторона базы (верхней плиты)
# E - сторона платформы (нижней плиты)
# LF - длина плеча (верхней перемычки)
# LE - длина рычага (нижней перемычки)
@profile
def inverse_kinematics(x, y, z, F, E, LF, LE):
    theta_1 = _calcAngleYZ(x, y, z, F, E, LF, LE)
    # theta_2 = _calcAngleYZ(*rotate2D(x, y, 120), z, F, E, LF, LE)  # rotate coords to +120 deg
    # theta_3 = _calcAngleYZ(*rotate2D(x, y, -120), z, F, E, LF, LE)  # rotate coords to -120 deg
    theta_2 = _calcAngleYZ(*rotate2Dsympy(x, y, cos120, sin120), z, F, E, LF, LE)  # rotate coords to +120 deg
    theta_3 = _calcAngleYZ(*rotate2Dsympy(x, y, cosMinus120, sinMinus120), z, F, E, LF, LE)  # rotate coords to -120 deg
    # if None in [theta_1, theta_2, theta_3]:
    #     return None
    return theta_1, theta_2, theta_3


if __name__ == '__main__':
    F0 = 1
    E0 = 0.3
    Lf0 = 0.7
    Le0 = 1.2

    x0 = 0
    y0 = 0
    z0 = -1  # должно быть < 0 (вниз, под базой)

    F, E, LF, LE = symbols('F, E, Lf, Le')
    x, y, z = symbols('x, y, z')
    t1, t2, t3 = symbols('t1, t2, t3')

    def calc_inverse():
        t1, t2, t3 = inverse_kinematics(x, y, z, F, E, LF, LE)
        print("")
        print("Inverse:")
        print(t1)
        print(t2)
        print(t3)
        return t1, t2, t3

    def calc_straight():
        x, y, z = straight_kinematics(t1, t2, t3, F, E, LF, LE)
        print("")
        print("Straight:")
        print(x)
        print(y)
        print(z)
        return x, y, z

    t1_anal, t2_anal, t3_anal = calc_inverse()
    x_anal, y_anal, z_anal = calc_straight()


    # -------- SPEEDS
    vx, vy, vz = symbols('vx, vy, vz')
    @profile
    def calc_inverse_speeds():
        def calc_anal_speed(t_anal):
            t_anal_dx = t_anal.diff(x)
            t_anal_dy = t_anal.diff(y)
            t_anal_dz = t_anal.diff(z)
            return t_anal_dx * vx + t_anal_dy * vy + t_anal_dz * vz  # Там точно не sqrt(dx^2 + dy^2 + dz^2)?

        w1 = calc_anal_speed(t1_anal)
        w2 = calc_anal_speed(t2_anal)
        w3 = calc_anal_speed(t3_anal)
        print("")
        print("Inverse speeds:")
        print("w1 =", w1)
        print("w2 =", w2)
        print("w3 =", w3)
        return w1, w2, w3

    w1, w2, w3 = symbols('w1, w2, w3')
    def calc_straight_speeds():
        def calc_anal_speed(coord_anal):
            coord_anal_dt1 = coord_anal.diff(t1)
            coord_anal_dt2 = coord_anal.diff(t2)
            coord_anal_dt3 = coord_anal.diff(t3)
            return coord_anal_dt1 * w1 + coord_anal_dt2 * w2 + coord_anal_dt3 * w3

        vx = calc_anal_speed(x_anal)
        vy = calc_anal_speed(y_anal)
        vz = calc_anal_speed(z_anal)
        print("")
        print("Straight speeds:")
        print("vx =", vx)
        print("vy =", vy)
        print("vz =", vz)
        return vx, vy, vz

    w1_anal, w2_anal, w3_anal = calc_inverse_speeds()
    vx_anal, vy_anal, vz_anal = calc_straight_speeds()


    # -------- ACCELERATIONS
    ax, ay, az = symbols('ax, ay, az')
    def calc_inverse_accelerations():
        def calc_anal_acceleration(w_anal):
            w_anal_dvx = w_anal.diff(x, 2)
            w_anal_dvy = w_anal.diff(y, 2)
            w_anal_dvz = w_anal.diff(z, 2)
            return w_anal_dvx * ax + w_anal_dvy * ay + w_anal_dvz * az

        e1 = calc_anal_acceleration(t1_anal)
        e2 = calc_anal_acceleration(t2_anal)
        e3 = calc_anal_acceleration(t3_anal)
        print("")
        print("Inverse accelerations:")
        with open('./out-inverse.txt', 'w') as f:
            f.write(str(e1) + ', \\\n')
            f.write('           ' + str(e2) + ', \\\n')
            f.write('           ' + str(e3) + '\n')
        print("Written to ./out-inverse.txt")
        return e1, e2, e3


    # TODO: make accelerations right
    e1, e2, e3 = symbols('e1, e2, e3')
    def calc_straight_accelerations():
        def calc_anal_acceleration(coord_anal):
            coord_anal_dw1 = coord_anal.diff(t1, 2)
            coord_anal_dw2 = coord_anal.diff(t2, 2)
            coord_anal_dw3 = coord_anal.diff(t3, 2)
            return coord_anal_dw1 * e1 + coord_anal_dw2 * e2 + coord_anal_dw3 * e3

        ax = calc_anal_acceleration(x_anal)
        ay = calc_anal_acceleration(y_anal)
        az = calc_anal_acceleration(z_anal)

        print("")
        print("Straight accelerations:")
        with open('./out-straight.txt', 'w') as f:
            f.write(str(ax) + ', \\\n')
            f.write('           ' + str(ay) + ', \\\n')
            f.write('           ' + str(az) + '\n')
        print("Written to ./out-straight.txt")
        return ax, ay, az

    e1_anal, e2_anal, e3_anal = calc_inverse_accelerations()
    ax_anal, ay_anal, az_anal = calc_straight_accelerations()
