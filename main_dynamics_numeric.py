import math
import time
from json.encoder import INFINITY

import numpy as np
import sympy

from src.kinematics import inverse_kinematics, straight_kinematics
from src.ui import UI


def rad2deg(val: float):
    return val / math.pi * 180
def deg2rad(val: float):
    return val / 180 * math.pi

if __name__ == '__main__':
    F = 1
    E = 0.3
    Lf = 0.7
    Le = 1.2
    Mlf = sympy.symbols('Mlf')
    Mle = sympy.symbols('Mle')
    Me = sympy.symbols('Me')
    # Mlf = 0.5
    # Mle = 1
    # Me = 20.3
    g = 9.8

    tMin = 0
    tMax = 10
    steps = 100
    t = np.linspace(tMin, tMax, steps)
    rTrajectory = 0.5
    zStartTrajectory = -0.5
    zEndTrajectory = -1  # должно быть < 0 (вниз, под базой)
    rps = 0.4
    xArr = np.array(list(map(lambda t: np.sin(t / (1 / rps) * np.pi * 2) * rTrajectory, t)))
    yArr = np.array(list(map(lambda t: np.cos(t / (1 / rps) * np.pi * 2) * rTrajectory, t)))
    zArr = np.linspace(zStartTrajectory, zEndTrajectory, steps)
    print("ℹ️ Time from:", tMin, "to", tMax)
    print("ℹ️ Diff steps:", steps)

    # Get speeds and accelerations on all trajectory
    dxArr = np.diff(xArr)
    dyArr = np.diff(yArr)
    dzArr = np.diff(zArr)
    ddxArr = np.diff(xArr, 2)
    ddyArr = np.diff(yArr, 2)
    ddzArr = np.diff(zArr, 2)
    t1Arr = []
    t2Arr = []
    t3Arr = []
    for step in range(steps):
        t1Cur, t2Cur, t3Cur = inverse_kinematics(xArr[step], yArr[step], zArr[step], F, E, Lf, Le)
        t1Arr.append(t1Cur)
        t2Arr.append(t2Cur)
        t3Arr.append(t3Cur)
    t1Arr = np.array(t1Arr)
    t2Arr = np.array(t2Arr)
    t3Arr = np.array(t3Arr)
    dt1Arr = np.diff(t1Arr)
    dt2Arr = np.diff(t2Arr)
    dt3Arr = np.diff(t3Arr)
    ddt1Arr = np.diff(t1Arr, 2)
    ddt2Arr = np.diff(t2Arr, 2)
    ddt3Arr = np.diff(t3Arr, 2)
    print("✅ All speeds and accelerations calculated")

    # maxDS = -INFINITY
    # maxDSMeta = []
    # maxDDS = -INFINITY
    # maxDDSMeta = []
    # maxDT = -INFINITY
    # maxDTMeta = []
    # maxDDT = -INFINITY
    # maxDDTMeta = []
    # maxTauForMlf = -INFINITY * Mlf
    # maxTauForMlfMeta = []
    # maxTauForMle = -INFINITY * Mlf
    # maxTauForMleMeta = []
    # maxTauForMe = -INFINITY * Mlf
    # maxTauForMeMeta = []
    # tau1 = []
    # tau2 = []
    # tau3 = []
    # for step in range(steps - 2): # -2 because work with 2nd diff
    #     if step % (steps // 10) == 0:
    #         print(step, "of", steps, "...")
    #     x, y, z = xArr[step], yArr[step], zArr[step]
    #     dx, dy, dz = dxArr[step], dyArr[step], dzArr[step]
    #     ddx, ddy, ddz = ddxArr[step], ddyArr[step], ddzArr[step]
    #     t1, t2, t3 = t1Arr[step], t2Arr[step], t3Arr[step]
    #     dt1, dt2, dt3 = dt1Arr[step], dt2Arr[step], dt3Arr[step]
    #     ddt1, ddt2, ddt3 = ddt1Arr[step], ddt2Arr[step], ddt3Arr[step]
    #
    #     dr = (E-F)/2 * np.tan(30 / 180*np.pi)
    #
    #     def calcFL(phi, theta):
    #         FactorX = x + np.cos(phi)*(dr - Lf*np.cos(theta))
    #         FactorY = y + np.sin(phi)*(dr - Lf*np.cos(theta))
    #         FactorZ = z - Lf*np.sin(theta)
    #         Fx = (Me + 3*Mle) * ddx / 2
    #         Fy = (Me + 3*Mle) * ddy / 2
    #         Fz = (Me + 3*Mle) * (ddz - g) / 2
    #         return np.dot([FactorX, FactorY, FactorZ], [Fx, Fy, Fz])
    #
    #     first1 = (Mlf/3 + Mle) * Lf * ddt1
    #     first2 = (Mlf/3 + Mle) * Lf * ddt2
    #     first3 = (Mlf/3 + Mle) * Lf * ddt3
    #     second1 = (Mlf/2 + Mle) * g * Lf * np.cos(t1)
    #     second2 = (Mlf/2 + Mle) * g * Lf * np.cos(t2)
    #     second3 = (Mlf/2 + Mle) * g * Lf * np.cos(t3)
    #     third1 = -2 * Lf * calcFL(0, t1) * ((x * np.cos(0) + y * np.sin(0) + dr) * np.sin(t1) - z * np.cos(t1))
    #     third2 = -2 * Lf * calcFL(-120 / 180 * np.pi, t2) * ((x * np.cos(-120 / 180 * np.pi) + y * np.sin(-120 / 180 * np.pi) + dr) * np.sin(t2) - z * np.cos(t2))
    #     third3 = -2 * Lf * calcFL(120 / 180 * np.pi, t3) * ((x * np.cos(120 / 180 * np.pi) + y * np.sin(120 / 180 * np.pi) + dr) * np.sin(t3) - z * np.cos(t3))
    #
    #     tau1.append(first1 + second1 + third1)
    #     tau2.append(first2 + second2 + third2)
    #     tau3.append(first3 + second3 + third3)
    #
    #     stepMeta = [step, x, y, z, dx, dy, dz, ddx, ddy, ddz]
    #     if np.linalg.norm([dx, dy, dz]) > maxDS:
    #         maxDS = np.linalg.norm([dx, dy, dz])
    #         maxDSMeta = stepMeta
    #     if np.linalg.norm([ddx, ddy, ddz]) > maxDDS:
    #         maxDDS = np.linalg.norm([ddx, ddy, ddz])
    #         maxDDSMeta = stepMeta
    #     if sympy.Max(dt1, dt2, dt3) > maxDT:
    #         maxDT = sympy.Max(dt1, dt2, dt3)
    #         maxDTMeta = stepMeta
    #     if sympy.Max(ddt1, ddt2, ddt3) > maxDDT:
    #         maxDDT = sympy.Max(ddt1, ddt2, ddt3)
    #         maxDDTMeta = stepMeta
    #
    #     maxTauForMlfSubs = maxTauForMlf.subs({Mlf: 999999, Mle: 0.00001, Me: 0.00001})
    #     maxTauForMleSubs = maxTauForMle.subs({Mlf: 0.00001, Mle: 999999, Me: 0.00001})
    #     maxTauForMeSubs = maxTauForMe.subs({Mlf: 0.00001, Mle: 0.00001, Me: 999999})
    #     def checkTau(tau):
    #         global maxTauForMlf, maxTauForMle, maxTauForMe, maxTauForMlfMeta, maxTauForMleMeta, maxTauForMeMeta
    #         tauForMlfSubs = tau.subs({Mlf: 999999, Mle: 0.00001, Me: 0.00001})
    #         tauForMleSubs = tau.subs({Mlf: 0.00001, Mle: 999999, Me: 0.00001})
    #         tauForMeSubs = tau.subs({Mlf: 0.00001, Mle: 0.00001, Me: 999999})
    #         if tauForMlfSubs > maxTauForMlfSubs:
    #             maxTauForMlf = tau
    #             maxTauForMlfMeta = stepMeta
    #         if tauForMleSubs > maxTauForMleSubs:
    #             maxTauForMle = tau
    #             maxTauForMleMeta = stepMeta
    #         if tauForMeSubs > maxTauForMeSubs:
    #             maxTauForMe = tau
    #             maxTauForMeMeta = stepMeta
    #     checkTau(tau1[step])
    #     checkTau(tau2[step])
    #     checkTau(tau3[step])
    #
    # def printMeta(meta):
    #     print(
    #         "|", [meta[0]],
    #         "== Point:", (np.round(meta[1], 2), np.round(meta[2], 2), np.round(meta[3], 2)),
    #         "== Speeds:", [np.round(meta[4], 2), np.round(meta[5], 2), np.round(meta[6], 2)],
    #         "== Accelerations:", (np.round(meta[7], 2), np.round(meta[8], 2), np.round(meta[9], 2))
    #     )
    #
    # print("👉 Max speed:", maxDS)
    # printMeta(maxDSMeta)
    # print("👉 Max acceleration:", maxDDS)
    # printMeta(maxDDSMeta)
    # print("👉 Max angle speed:", maxDT)
    # printMeta(maxDTMeta)
    # print("👉 Max angle acceleration:", maxDDT)
    # printMeta(maxDDTMeta)
    # print("👉 Max moments for each mass:")
    # print("- Mlf:", maxTauForMlf)
    # printMeta(maxTauForMlfMeta)
    # print("- Mle:", maxTauForMle)
    # printMeta(maxTauForMleMeta)
    # print("- Me:", maxTauForMe)
    # printMeta(maxTauForMeMeta)


    def showAnimationInUI():
        slowDownFactor = 3
        tDiffs = np.diff(t)
        print(tDiffs)
        for step in range(steps-2):
            x, y, z = xArr[step], yArr[step], zArr[step]
            dx, dy, dz = dxArr[step], dyArr[step], dzArr[step]
            ddx, ddy, ddz = ddxArr[step], ddyArr[step], ddzArr[step]
            t1, t2, t3 = t1Arr[step], t2Arr[step], t3Arr[step]
            dt1, dt2, dt3 = dt1Arr[step], dt2Arr[step], dt3Arr[step]
            ddt1, ddt2, ddt3 = ddt1Arr[step], ddt2Arr[step], ddt3Arr[step]

            time.sleep(tDiffs[step] * slowDownFactor)
            ui.setAngles(t1, t2, t3, False)
            ui.setSpeeds(dt1, dt2, dt3, False)
            ui.setAccelerations(ddt1, ddt2, ddt3, True)
    ui = UI()
    ui.start(showAnimationInUI)

