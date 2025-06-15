import math

from src.acceleration_kinematics import inverse_acceleration_kinematics, straight_acceleration_kinematics
from src.kinematics import inverse_kinematics, straight_kinematics
from src.speed_kinematics import inverse_speed_kinematics, straight_speed_kinematics

def rad2deg(val: float):
    return val / math.pi * 180
def deg2rad(val: float):
    return val / 180 * math.pi

if __name__ == '__main__':
    F = 1
    E = 0.3
    Lf = 0.7
    Le = 1.2

    print("\n--- Positions ---")
    x = -0.2
    y = 0.5
    z = -1  # должно быть < 0 (вниз, под базой)
    print("| Position (fixed):", x, y, z)
    t1, t2, t3 = inverse_kinematics(x, y, z, F, E, Lf, Le)
    print(">> Angles:", round(rad2deg(t1), 2), round(rad2deg(t2), 2), round(rad2deg(t3), 2))
    _x, _y, _z = straight_kinematics(t1, t2, t3, F, E, Lf, Le)
    print("Position (reversed):", round(_x, 2), round(_y, 2), round(_z, 2))

    print("\n--- Speeds ---")
    x = 0
    y = 0
    z = -1  # должно быть < 0 (вниз, под базой)
    vx = 1
    vy = 0
    vz = 0
    print("| Ending position (fixed):", x, y, z)
    print("| Ending speeds (fixed):", vx, vy, vz)
    t1, t2, t3 = inverse_kinematics(x, y, z, F, E, Lf, Le)
    print(">> Angular positions:", round(rad2deg(t1), 2), round(rad2deg(t2), 2), round(rad2deg(t3), 2))
    w1, w2, w3 = inverse_speed_kinematics(vx, vy, vz, x, y, z, F, E, Lf, Le)
    print(">> Angular speeds:", round(w1, 2), round(w2, 2), round(w3, 2))
    _sx, _sy, _sz = straight_speed_kinematics(w1, w2, w3, t1, t2, t3, F, E, Lf, Le)
    print("Ending speeds (reversed):", round(_sx, 2), round(_sy, 2), round(_sz, 2))

    print("\n--- Accelerations ---")
    x = 0
    y = 0
    z = -1  # должно быть < 0 (вниз, под базой)
    vx = 0
    vy = 0
    vz = 0
    ax = 0
    ay = 1
    az = 0
    print("| Ending position (fixed):", x, y, z)
    print("| Ending speeds (fixed): ", vx, vy, vz)
    print("| Ending accelerations (fixed):", ax, ay, az)
    t1, t2, t3 = inverse_kinematics(x, y, z, F, E, Lf, Le)
    print(">> Angular positions:", round(rad2deg(t1), 2), round(rad2deg(t2), 2), round(rad2deg(t3), 2))
    w1, w2, w3 = inverse_speed_kinematics(vx, vy, vz, x, y, z, F, E, Lf, Le)
    print(">> Angular speeds:", round(w1, 2), round(w2, 2), round(w3, 2))
    e1, e2, e3 = inverse_acceleration_kinematics(ax, ay, az, vx, vy, vz, x, y, z, F, E, Lf, Le)
    print(">> Angular accelerations:", round(e1, 2), round(e2, 2), round(e3, 2))
    _ax, _ay, _az = straight_acceleration_kinematics(e1, e2, e3, w1, w2, w3, x, y, z, F, E, Lf, Le)
    print("Ending accelerations (reversed):", round(_ax, 2), round(_ay, 2), round(_az, 2))

