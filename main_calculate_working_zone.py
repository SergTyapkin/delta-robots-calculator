from math import *
from progress.bar import IncrementalBar

from show_points_cloud import show_points_cloud, approx
from src.kinematics import inverse_kinematics
from utils import Point, save_points_cloud_to_file, linspace


def get_working_zone_points_cloud_for_configuration(X, Y, Z, F, E, LF, LE):
    points_cloud = []
    progressbar = IncrementalBar('Points', max=len(X) * len(Y) * len(Z))
    step = progressbar.max / 20
    i = 0
    for x in X:
        for y in Y:
            for z in Z:
                res = inverse_kinematics(x, y, z, F, E, LF, LE)
                # print(x, y, z, res)
                if res is not None:
                    points_cloud.append(Point(x, y, z))
                i += 1
                if i % step == 0:
                    progressbar.goto(progressbar.index + step)
    progressbar.finish()
    return points_cloud


def calculate_approximations_params(points):
    hmax = -inf
    zmin = inf
    for point in points:
        if approx(point.x, 0) and approx(point.y, 0):
            hmax = max(hmax, point.z)
        zmin = min(zmin, point.z)
    hmin = zmin + (hmax - zmin) / pi
    wmax = -inf
    for point in points:
        if approx(point.z, hmin):
            wmax = max(wmax, sqrt(point.x * point.x + point.y * point.y))
    print("| h_max:", hmax)
    print("| h_min:", hmin)
    print("| h_delta:", hmax - hmin)
    print("| z_min:", zmin)
    print("| D: ", wmax * 2)
    print("| V:", wmax * wmax * pi * (hmax - hmin))


if __name__ == '__main__':
    F = 1
    E = 0.3
    # LF = 0.7
    # LE = 1.2

    # STEP_CALC = 0.02
    STEP_CALC = 0.1

    X = linspace(-F * 1.5, +F * 1.5, STEP_CALC)
    Y = linspace(-F * 1.5, +F * 1.5, STEP_CALC)
    Z = linspace(-2 * F, F * 0.2, STEP_CALC)

    # LFs = linspace(0.7, 0.90001, 0.1)
    # LEs = linspace(1.2, 1.40001, 0.1)
    LFs = [0.7]
    LEs = [1.2]

    # progressbar = IncrementalBar('Points', max=len(X) * len(Y) * len(Z))
    for LF in LFs:
        for LE in LEs:
            if LE <= LF:
                continue
            points_cloud = get_working_zone_points_cloud_for_configuration(X, Y, Z, F, E, LF, LE)
            save_points_cloud_to_file(points_cloud)
            show_points_cloud(f'_LF={LF}_LE={LE}', f'LF={LF}; LE={LE}')
            print(f"LF={LF}; LE={LE}: ")
            calculate_approximations_params(points_cloud)
            # progressbar.next()
    # progressbar.finish()
