from math import cos, sin, tan, pi, sqrt, atan
from src.utils import rotate2D, Point
from line_profiler import profile
import os
os.environ["LINE_PROFILE"] = "0" # Enable or disable line_profiler

tan30 = tan(30 / 180 * pi)
tan60 = tan(60 / 180 * pi)
sin30 = sin(30 / 180 * pi)


# ----------- ПРЯМАЯ КИНЕМАТИКА -------------
# Прямая кинематика: (theta1, theta2, theta3) в градусах -> (x, y, z) рабочего органа
# theta_1 - угол привода, расположенного на оси Y
# theta_2 - угол привода, повернутого от первого против часовой стрелки на 120 градусов
# theta_3 - угол привода, повернутого от первого по часовой стрелке на 120 градусов
# F - сторона базы (верхней плиты)
# E - сторона платформы (нижней плиты)
# Lf - длина плеча (верхней перемычки)
# Le - длина рычага (нижней перемычки)
@profile
def straight_kinematics(theta_1, theta_2, theta_3, F, E, Lf, Le):
    t = (F - E) * tan30 / 2

    y1 = -(t + Lf * cos(theta_1))
    x1 = 0
    z1 = -Lf * sin(theta_1)

    y2 = (t + Lf * cos(theta_2)) * sin30
    x2 = y2 * tan60
    z2 = -Lf * sin(theta_2)

    y3 = (t + Lf * cos(theta_3)) * sin30
    x3 = -y3 * tan60
    z3 = -Lf * sin(theta_3)

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
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - Le * Le)

    # дискриминант
    d = b * b - 4 * a * c
    if d < 0:
        return None  # несуществующая позиция

    z0 = -0.5 * (b + sqrt(d)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm
    return x0, y0, z0

@profile
def straight_kinematics_lowperf(t1, t2, t3, F, E, Lf, Le):
    return (((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))*(-1.0*Lf*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2*sin(t1) - 1.0*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))*((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2) - 0.5*sqrt(-((-Le**2 + Lf**2*sin(t1)**2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + ((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2)**2 + (-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)))**2)*(4*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + 4*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))**2 + 4*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2) + (2*Lf*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2*sin(t1) + 2*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))*((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2) + 2*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))))**2) - 1.0*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))))/(((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + (-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))**2 + (-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2) + (Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2)/(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)), \
           (-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 + (-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))*(-1.0*Lf*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2*sin(t1) - 1.0*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))*((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2) - 0.5*sqrt(-((-Le**2 + Lf**2*sin(t1)**2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + ((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2)**2 + (-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)))**2)*(4*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + 4*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))**2 + 4*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2) + (2*Lf*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2*sin(t1) + 2*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))*((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2) + 2*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))))**2) - 1.0*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))))/(((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + (-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))**2 + (-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2))/(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)),  \
           (-1.0*Lf*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2*sin(t1) - 1.0*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))*((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2) - 0.5*sqrt(-((-Le**2 + Lf**2*sin(t1)**2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + ((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2)**2 + (-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)))**2)*(4*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + 4*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))**2 + 4*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2) + (2*Lf*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2*sin(t1) + 2*((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))*((Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2) + 2*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))))**2) - 1.0*(-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t3)**2 + sin30**2*tan60**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t3) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(-Lf**2*sin(t1)**2 + Lf**2*sin(t2)**2 + sin30**2*tan60**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 + sin30**2*(Lf*cos(t2) + tan30*(-E + F)/2)**2 - (-Lf*cos(t1) - tan30*(-E + F)/2)**2)/2 - (-Lf*cos(t1) - tan30*(-E + F)/2)*(-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))))/(((Lf*sin(t1) - Lf*sin(t2))*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - (Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2 + (-sin30*tan60*(-Lf*sin(t1) + Lf*sin(t2))*(Lf*cos(t3) + tan30*(-E + F)/2) + sin30*tan60*(Lf*sin(t1) - Lf*sin(t3))*(Lf*cos(t2) + tan30*(-E + F)/2))**2 + (-sin30*tan60*(Lf*cos(t2) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t3) + tan30*(-E + F)/2) + tan30*(-E + F)/2) - sin30*tan60*(Lf*cos(t3) + tan30*(-E + F)/2)*(Lf*cos(t1) + sin30*(Lf*cos(t2) + tan30*(-E + F)/2) + tan30*(-E + F)/2))**2)


# ----------- ОБРАТНАЯ КИНЕМАТИКА -------------
# вспомогательная функция, расчет угла theta1 (в плоскости YZ)
# (в градусах поворота относительно горизонтального луча по направлению от базы)
def _calcAngLeYZ(x0, y0, z0, F, E, Lf, Le):
    y1 = -F / 2 * tan30  # сдвигаем центр к краю верхней базы (к приводу)
    y0 -= E / 2 * tan30  # сдвигаем центр к краю нижней платформы
    # z = a + b*y
    if z0 == 0:
        return None  # несуществующая точка
    a = (x0 * x0 + y0 * y0 + z0 * z0 + Lf * Lf - Le * Le - y1 * y1) / (2 * z0)
    b = (y1 - y0) / z0
    # дискриминант
    d = -(a + b * y1) * (a + b * y1) + Lf * (b * b * Lf + Lf)
    # print(a, b, y1, y0, E, F, d)
    if d < 0:
        return None  # несуществующая точка
    yj = (y1 - a * b - sqrt(d)) / (b * b + 1)  # выбираем внешнюю точку
    zj = a + b * yj
    return atan(-zj / (y1 - yj)) / pi * 180 + (180 if (yj > y1) else 0)


# Обратная кинематика: (x0, y0, z0) -> (theta1, theta2, theta3) в градусах
# При этом theta_1 - угол привода, расположенного на оси Y
# theta_2 - угол привода, повернутого от первого против часовой стрелки на 120 градусов
# theta_3 - угол привода, повернутого от первого по часовой стрелке на 120 градусов
# ---
# x, y, z - координаты точки для поиска углов
# F - сторона базы (верхней плиты)
# E - сторона платформы (нижней плиты)
# Lf - длина плеча (верхней перемычки)
# Le - длина рычага (нижней перемычки)
@profile
def inverse_kinematics(x, y, z, F, E, Lf, Le):
    theta_1 = _calcAngLeYZ(x, y, z, F, E, Lf, Le)
    theta_2 = _calcAngLeYZ(*rotate2D(x, y, 120), z, F, E, Lf, Le)  # rotate coords to +120 deg
    theta_3 = _calcAngLeYZ(*rotate2D(x, y, -120), z, F, E, Lf, Le)  # rotate coords to -120 deg
    if None in [theta_1, theta_2, theta_3]:
        return None
    return theta_1 / 180 * pi, theta_2 / 180 * pi, theta_3 / 180 * pi

@profile
def inverse_kinematics_lowperf(x, y, z, F, E, Lf, Le):
    return atan((-(-F**2*tan30**2/4 - Le**2 + Lf**2 + x**2 + z**2 + (-E*tan30/2 + y)**2)/(2*z) - (E*tan30/2 - F*tan30/2 - y)*(-F*tan30/2 - sqrt(Lf*(Lf + Lf*(E*tan30/2 - F*tan30/2 - y)**2/z**2) + (-F*tan30*(E*tan30/2 - F*tan30/2 - y)/(2*z) + (-F**2*tan30**2/4 - Le**2 + Lf**2 + x**2 + z**2 + (-E*tan30/2 + y)**2)/(2*z))*(F*tan30*(E*tan30/2 - F*tan30/2 - y)/(2*z) - (-F**2*tan30**2/4 - Le**2 + Lf**2 + x**2 + z**2 + (-E*tan30/2 + y)**2)/(2*z))) - (E*tan30/2 - F*tan30/2 - y)*(-F**2*tan30**2/4 - Le**2 + Lf**2 + x**2 + z**2 + (-E*tan30/2 + y)**2)/(2*z**2))/(z*(1 + (E*tan30/2 - F*tan30/2 - y)**2/z**2)))/(-F*tan30/2 - (-F*tan30/2 - sqrt(Lf*(Lf + Lf*(E*tan30/2 - F*tan30/2 - y)**2/z**2) + (-F*tan30*(E*tan30/2 - F*tan30/2 - y)/(2*z) + (-F**2*tan30**2/4 - Le**2 + Lf**2 + x**2 + z**2 + (-E*tan30/2 + y)**2)/(2*z))*(F*tan30*(E*tan30/2 - F*tan30/2 - y)/(2*z) - (-F**2*tan30**2/4 - Le**2 + Lf**2 + x**2 + z**2 + (-E*tan30/2 + y)**2)/(2*z))) - (E*tan30/2 - F*tan30/2 - y)*(-F**2*tan30**2/4 - Le**2 + Lf**2 + x**2 + z**2 + (-E*tan30/2 + y)**2)/(2*z**2))/(1 + (E*tan30/2 - F*tan30/2 - y)**2/z**2))) + pi, \
           atan((-(-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 - sqrt(3)*x - 1.0*y)**2/4)/(2*z) - (-F*tan30/2 - sqrt(Lf*(Lf + Lf*(E*tan30 - F*tan30 + sqrt(3)*x + 1.0*y)**2/(4*z**2)) + (-F*tan30*(E*tan30/2 - F*tan30/2 + sqrt(3)*x/2 + 0.5*y)/(2*z) + (-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 - sqrt(3)*x - 1.0*y)**2/4)/(2*z))*(F*tan30*(E*tan30/2 - F*tan30/2 + sqrt(3)*x/2 + 0.5*y)/(2*z) - (-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 - sqrt(3)*x - 1.0*y)**2/4)/(2*z))) - (E*tan30/2 - F*tan30/2 + sqrt(3)*x/2 + 0.5*y)*(-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 - sqrt(3)*x - 1.0*y)**2/4)/(2*z**2))*(E*tan30/2 - F*tan30/2 + sqrt(3)*x/2 + 0.5*y)/(z*(1 + ((E*tan30 - F*tan30 + sqrt(3)*x + 1.0*y)**2/4)/z**2)))/(-F*tan30/2 - (-F*tan30/2 - sqrt(Lf*(Lf + Lf*(E*tan30 - F*tan30 + sqrt(3)*x + 1.0*y)**2/(4*z**2)) + (-F*tan30*(E*tan30/2 - F*tan30/2 + sqrt(3)*x/2 + 0.5*y)/(2*z) + (-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 - sqrt(3)*x - 1.0*y)**2/4)/(2*z))*(F*tan30*(E*tan30/2 - F*tan30/2 + sqrt(3)*x/2 + 0.5*y)/(2*z) - (-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 - sqrt(3)*x - 1.0*y)**2/4)/(2*z))) - (E*tan30/2 - F*tan30/2 + sqrt(3)*x/2 + 0.5*y)*(-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 - sqrt(3)*x - 1.0*y)**2/4)/(2*z**2))/(1 + ((E*tan30 - F*tan30 + sqrt(3)*x + 1.0*y)**2/4)/z**2))) + pi, \
           atan((-(-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (-sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 + sqrt(3)*x - 1.0*y)**2/4)/(2*z) - (-F*tan30/2 - sqrt(Lf*(Lf + Lf*(E*tan30 - F*tan30 - sqrt(3)*x + 1.0*y)**2/(4*z**2)) + (-F*tan30*(E*tan30/2 - F*tan30/2 - sqrt(3)*x/2 + 0.5*y)/(2*z) + (-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (-sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 + sqrt(3)*x - 1.0*y)**2/4)/(2*z))*(F*tan30*(E*tan30/2 - F*tan30/2 - sqrt(3)*x/2 + 0.5*y)/(2*z) - (-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (-sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 + sqrt(3)*x - 1.0*y)**2/4)/(2*z))) - (E*tan30/2 - F*tan30/2 - sqrt(3)*x/2 + 0.5*y)*(-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (-sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 + sqrt(3)*x - 1.0*y)**2/4)/(2*z**2))*(E*tan30/2 - F*tan30/2 - sqrt(3)*x/2 + 0.5*y)/(z*(1 + ((E*tan30 - F*tan30 - sqrt(3)*x + 1.0*y)**2/4)/z**2)))/(-F*tan30/2 - (-F*tan30/2 - sqrt(Lf*(Lf + Lf*(E*tan30 - F*tan30 - sqrt(3)*x + 1.0*y)**2/(4*z**2)) + (-F*tan30*(E*tan30/2 - F*tan30/2 - sqrt(3)*x/2 + 0.5*y)/(2*z) + (-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (-sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 + sqrt(3)*x - 1.0*y)**2/4)/(2*z))*(F*tan30*(E*tan30/2 - F*tan30/2 - sqrt(3)*x/2 + 0.5*y)/(2*z) - (-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (-sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 + sqrt(3)*x - 1.0*y)**2/4)/(2*z))) - (E*tan30/2 - F*tan30/2 - sqrt(3)*x/2 + 0.5*y)*(-F**2*tan30**2/4 - Le**2 + Lf**2 + z**2 + (-sqrt(3)*y - 1.0*x)**2/4 + (-E*tan30 + sqrt(3)*x - 1.0*y)**2/4)/(2*z**2))/(1 + ((E*tan30 - F*tan30 - sqrt(3)*x + 1.0*y)**2/4)/z**2))) + pi



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
    print('theta 1:', theta_1/pi*180 % 180)
    print('theta 2:', theta_2/pi*180 % 180)
    print('theta 3:', theta_3/pi*180 % 180)
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
