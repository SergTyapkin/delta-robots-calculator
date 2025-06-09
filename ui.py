from dataclasses import dataclass
from math import *

from src.acceleration_kinematics import straight_acceleration_kinematics
from src.speed_kinematics import straight_speed_kinematics
from vendor.tiny_3d_engine.scene3d import Scene3D
from vendor.tiny_3d_engine.engine import Engine3D
import numpy as np

from src.kinematics import straight_kinematics, inverse_kinematics


def radToDeg(val: float):
    return val / pi * 180


def degToRad(val: float):
    return val / 180 * pi


def Rx(angle: float):
    return np.matrix([
        [1, 0, 0],
        [0, cos(angle), -sin(angle)],
        [0, sin(angle), cos(angle)]
    ])


def Ry(angle: float):
    return np.matrix([
        [cos(angle), 0, -sin(angle)],
        [0, 1, 0],
        [sin(angle), 0, cos(angle)]
    ])


def Rz(angle: float):
    return np.matrix([
        [cos(angle), -sin(angle), 0],
        [sin(angle), cos(angle), 0],
        [0, 0, 1]
    ])


def rotate3D(p, center=[.0, .0, .0], rx=.0, ry=.0, rz=.0):
    rotateMatrix = Rz(rz) * Ry(ry) * Rx(rx)
    return np.array(((np.array(p) - np.array(center)) * rotateMatrix).tolist()[0])


def triangleHeightBySide(side: float):
    return side * cos(degToRad(30))


def rotate(side: float):
    return side * cos(degToRad(30))


def triangleFromHeightPointToSide(side: float):
    return side / 2 * tan(degToRad(30))


def triangleFromHeightPointToVertex(side: float):
    return side / 2 / cos(degToRad(30))


def getLength(pointOrVector, fromPointIfPoint=[0, 0, 0]):
    return sqrt(sum((np.array(pointOrVector) - fromPointIfPoint) ** 2))


class UI:
    def __init__(self):
        self.scene = Scene3D("World")
        self.update()

        ANGLE_DELTA = degToRad(5)
        SPEED_DELTA = 0.1
        ACCELERATION_DELTA = 0.1
        POSITION_DELTA = 0.1
        self.engine = Engine3D(self.scene, background="#1e1f22", shading="radial", keyHandlers={
            "!": lambda _: self.setAngles(self.State.t1 + ANGLE_DELTA, self.State.t2, self.State.t3),
            "1": lambda _: self.setAngles(self.State.t1 - ANGLE_DELTA, self.State.t2, self.State.t3),
            "@": lambda _: self.setAngles(self.State.t1, self.State.t2 + ANGLE_DELTA, self.State.t3),
            "2": lambda _: self.setAngles(self.State.t1, self.State.t2 - ANGLE_DELTA, self.State.t3),
            "#": lambda _: self.setAngles(self.State.t1, self.State.t2, self.State.t3 + ANGLE_DELTA),
            "3": lambda _: self.setAngles(self.State.t1, self.State.t2, self.State.t3 - ANGLE_DELTA),

            "Q": lambda _: self.setSpeeds(self.State.w1 + SPEED_DELTA, self.State.w2, self.State.w3),
            "q": lambda _: self.setSpeeds(self.State.w1 - SPEED_DELTA, self.State.w2, self.State.w3),
            "W": lambda _: self.setSpeeds(self.State.w1, self.State.w2 + SPEED_DELTA, self.State.w3),
            "w": lambda _: self.setSpeeds(self.State.w1, self.State.w2 - SPEED_DELTA, self.State.w3),
            "E": lambda _: self.setSpeeds(self.State.w1, self.State.w2, self.State.w3 + SPEED_DELTA),
            "e": lambda _: self.setSpeeds(self.State.w1, self.State.w2, self.State.w3 - SPEED_DELTA),

            "A": lambda _: self.setAccelerations(self.State.e1 + ACCELERATION_DELTA, self.State.e2, self.State.e3),
            "a": lambda _: self.setAccelerations(self.State.e1 - ACCELERATION_DELTA, self.State.e2, self.State.e3),
            "S": lambda _: self.setAccelerations(self.State.e1, self.State.e2 + ACCELERATION_DELTA, self.State.e3),
            "s": lambda _: self.setAccelerations(self.State.e1, self.State.e2 - ACCELERATION_DELTA, self.State.e3),
            "D": lambda _: self.setAccelerations(self.State.e1, self.State.e2, self.State.e3 + ACCELERATION_DELTA),
            "d": lambda _: self.setAccelerations(self.State.e1, self.State.e2, self.State.e3 - ACCELERATION_DELTA),

            "x": lambda _: self.setEndPosition(self.points.E.center[0] + POSITION_DELTA, self.points.E.center[1], self.points.E.center[2]),
            "X": lambda _: self.setEndPosition(self.points.E.center[0] - POSITION_DELTA, self.points.E.center[1], self.points.E.center[2]),
            "y": lambda _: self.setEndPosition(self.points.E.center[0], self.points.E.center[1] + POSITION_DELTA, self.points.E.center[2]),
            "Y": lambda _: self.setEndPosition(self.points.E.center[0], self.points.E.center[1] - POSITION_DELTA, self.points.E.center[2]),
            "z": lambda _: self.setEndPosition(self.points.E.center[0], self.points.E.center[1], self.points.E.center[2] + POSITION_DELTA),
            "Z": lambda _: self.setEndPosition(self.points.E.center[0], self.points.E.center[1], self.points.E.center[2] - POSITION_DELTA),
        })
        self.engine.rotate("x", 45)
        self.engine.render()

    @dataclass
    class State:
        F = 1
        E = 0.3
        Lf = 0.7
        Le = 1.2

        t1 = 0
        t2 = 0
        t3 = 0

        w1 = 0
        w2 = 0
        w3 = 0

        e1 = 0
        e2 = 0
        e3 = 0

    @dataclass
    class points:
        @dataclass
        class F:
            center = None
            v1 = None
            v2 = None
            v3 = None

        @dataclass
        class E:
            center = None
            v1 = None
            v2 = None
            v3 = None

        @dataclass
        class LF1:
            start = None
            end = None

        @dataclass
        class LF2:
            start = None
            end = None

        @dataclass
        class LF3:
            start = None
            end = None

        @dataclass
        class LE1:
            start = None
            end = None

        @dataclass
        class LE2:
            start = None
            end = None

        @dataclass
        class LE3:
            start = None
            end = None

    def addTriangleByPoints(self, name: str, p1, p2, p3, color: str):
        self.scene.update(
            name=name,
            points=[p1, p2, p3],
            conn=[[0, 1, 2]],
            color=color,
        )

    def addLineByPoints(self, name: str, p1, p2, color: str, width=1):
        self.scene.update(
            name=name,
            points=[p1, p2],
            conn=[[0, 1]],
            color=color,
            width=width,
        )

    def addTriangle(self, name: str, triangle, color: str):
        self.addTriangleByPoints(name, triangle.v1, triangle.v2, triangle.v3, color)

    def addLine(self, name: str, line, color: str, width=1):
        self.addLineByPoints(name, line.start, line.end, color, width)

    # ---- AXIS
    def addAxes(self):
        self.scene.add_axes('zero')
        # Create my own axes
        # self.addLineByPoints(
        #     "Z-axis",
        #     [0, 0, 0.5],
        #     [0, 0, -1],
        #     "#0000ff",
        # )
        # self.addLineByPoints(
        #     "Y-axis",
        #     [0, -0.5, 0],
        #     [0, 1, 0],
        #     "#00ff00",
        # )
        # self.addLineByPoints(
        #     "X-axis",
        #     [-0.5, 0, 0],
        #     [1, 0, 0],
        #     "#ff0000",
        # )

    # ---- GEOMETRY
    def calculateDeltaRobotPoints(self, F, E, Lf, Le, theta1, theta2, theta3):
        self.points.F.center = np.array([0, 0, 0])
        self.points.F.v1 = np.array([0, triangleFromHeightPointToVertex(F), 0])
        self.points.F.v2 = rotate3D([0, triangleFromHeightPointToVertex(F), 0], rz=degToRad(120))
        self.points.F.v3 = rotate3D([0, triangleFromHeightPointToVertex(F), 0], rz=degToRad(-120))

        self.points.E.center = np.array(straight_kinematics(theta1, theta2, theta3, F, E, Lf, Le))
        self.points.E.v1 = self.points.E.center + [0, triangleFromHeightPointToVertex(E), 0]
        self.points.E.v2 = self.points.E.center + rotate3D([0, triangleFromHeightPointToVertex(E), 0], rz=degToRad(120))
        self.points.E.v3 = self.points.E.center + rotate3D([0, triangleFromHeightPointToVertex(E), 0],
                                                           rz=degToRad(-120))

        self.points.LF1.start = np.array([0, -triangleFromHeightPointToSide(F), 0])
        self.points.LF1.end = self.points.LF1.start + rotate3D([0, -Lf, 0], rx=-theta1)
        self.points.LE1.start = self.points.LF1.end
        self.points.LE1.end = self.points.E.center + [0, -triangleFromHeightPointToSide(E), 0]

        LFStartInY = np.array([0, -triangleFromHeightPointToSide(F), 0])
        LFendRotX = LFStartInY + rotate3D([0, -Lf, 0], rx=-theta2)
        self.points.LF2.end = rotate3D(LFendRotX, rz=degToRad(-120))
        self.points.LF2.start = rotate3D([0, -triangleFromHeightPointToSide(F), 0], rz=degToRad(-120))
        self.points.LE2.start = self.points.LF2.end
        self.points.LE2.end = self.points.E.center + rotate3D([0, -triangleFromHeightPointToSide(E), 0],
                                                              rz=degToRad(-120))

        LFStartInY = np.array([0, -triangleFromHeightPointToSide(F), 0])
        LFendRotX = LFStartInY + rotate3D([0, -Lf, 0], rx=-theta3)
        self.points.LF3.end = rotate3D(LFendRotX, rz=degToRad(120))
        self.points.LF3.start = rotate3D([0, -triangleFromHeightPointToSide(F), 0], rz=degToRad(120))
        self.points.LE3.start = self.points.LF3.end
        self.points.LE3.end = self.points.E.center + rotate3D([0, -triangleFromHeightPointToSide(E), 0],
                                                              rz=degToRad(120))

    def addDeltaRobotToScene(self):
        TOP_BASE_COLOR = "#ff0000"
        BOTTOM_BASE_COLOR = "#0000ff"
        DELTA_LINES_COLOR = "#ffff00"
        DELTA_LINE_WIDTHS = 3
        SPEEDS_LINES_COLOR = "#00ffff"
        SPEEDS_LINES_WIDTHS = 1
        ACCELERATIONS_LINES_COLOR = "#ff00ff"
        ACCELERATIONS_LINES_WIDTHS = 3
        self.addTriangle("TopBase", self.points.F, TOP_BASE_COLOR)

        self.addLine(f"Lf1_{round(getLength(self.points.LF1.start, self.points.LF1.end), 2)}", self.points.LF1,
                     DELTA_LINES_COLOR, DELTA_LINE_WIDTHS)
        self.addLine(f"Le1_{round(getLength(self.points.LE1.start, self.points.LE1.end), 2)}", self.points.LE1,
                     DELTA_LINES_COLOR, DELTA_LINE_WIDTHS)
        self.addLine(f"Lf2_{round(getLength(self.points.LF2.start, self.points.LF2.end), 2)}", self.points.LF2,
                     DELTA_LINES_COLOR, DELTA_LINE_WIDTHS)
        self.addLine(f"Le2_{round(getLength(self.points.LE2.start, self.points.LE2.end), 2)}", self.points.LE2,
                     DELTA_LINES_COLOR, DELTA_LINE_WIDTHS)
        self.addLine(f"Lf3_{round(getLength(self.points.LF3.start, self.points.LF3.end), 2)}", self.points.LF3,
                     DELTA_LINES_COLOR, DELTA_LINE_WIDTHS)
        self.addLine(f"Le3_{round(getLength(self.points.LE3.start, self.points.LE3.end), 2)}", self.points.LE3,
                     DELTA_LINES_COLOR, DELTA_LINE_WIDTHS)

        self.addTriangle(
            f"BottomBase_x:{round(self.points.E.center[0], 2)}_y:{round(self.points.E.center[1], 2)}_z:{round(self.points.E.center[2], 2)}",
            self.points.E, BOTTOM_BASE_COLOR)

        self.addLineByPoints(f"w1_{round(self.State.w1, 2)}", self.points.LF1.start,
                             self.points.LF1.start + [0, 0, -self.State.w1],
                             SPEEDS_LINES_COLOR, SPEEDS_LINES_WIDTHS)
        self.addLineByPoints(f"w2_{round(self.State.w2, 2)}", self.points.LF2.start,
                             self.points.LF2.start + [0, 0, -self.State.w2],
                             SPEEDS_LINES_COLOR, SPEEDS_LINES_WIDTHS)
        self.addLineByPoints(f"w3_{round(self.State.w3, 2)}", self.points.LF3.start,
                             self.points.LF3.start + [0, 0, -self.State.w3],
                             SPEEDS_LINES_COLOR, SPEEDS_LINES_WIDTHS)

        self.addLineByPoints(f"e1_{round(self.State.e1, 2)}", self.points.LF1.start,
                             self.points.LF1.start + [0, 0, -self.State.e1],
                             ACCELERATIONS_LINES_COLOR, ACCELERATIONS_LINES_WIDTHS)
        self.addLineByPoints(f"e2_{round(self.State.e2, 2)}", self.points.LF2.start,
                             self.points.LF2.start + [0, 0, -self.State.e2],
                             ACCELERATIONS_LINES_COLOR, ACCELERATIONS_LINES_WIDTHS)
        self.addLineByPoints(f"e3_{round(self.State.e3, 2)}", self.points.LF3.start,
                             self.points.LF3.start + [0, 0, -self.State.e3],
                             ACCELERATIONS_LINES_COLOR, ACCELERATIONS_LINES_WIDTHS)

        speed = straight_speed_kinematics(self.State.w1, self.State.w2, self.State.w3, self.State.t1, self.State.t2, self.State.t3, self.State.F, self.State.E, self.State.Lf, self.State.Le)
        self.addLineByPoints(f"speed_{round(getLength(speed), 2)}", self.points.E.center, self.points.E.center + speed,
                             SPEEDS_LINES_COLOR, SPEEDS_LINES_WIDTHS)
        acceleration = straight_acceleration_kinematics(self.State.e1, self.State.e2, self.State.e3,
                                                        self.State.w1, self.State.w2, self.State.w3,
                                                        self.State.t1, self.State.t2, self.State.t3,
                                                        self.State.F, self.State.E, self.State.Lf, self.State.Le)
        self.addLineByPoints(f"acceleration_{round(getLength(acceleration), 2)}", self.points.E.center,
                             self.points.E.center + acceleration,
                             ACCELERATIONS_LINES_COLOR, ACCELERATIONS_LINES_WIDTHS)

    def update(self):
        self.scene.clear()
        self.calculateDeltaRobotPoints(self.State.F, self.State.E, self.State.Lf, self.State.Le,
                                       self.State.t1, self.State.t2, self.State.t3)
        self.addDeltaRobotToScene()
        self.addAxes()

    def rerender(self):
        self.engine.clear()
        self.update()
        self.engine.render()

    def setAngles(self, theta1, theta2, theta3):
        def limitAngle(val: float):
            return max(-pi, min(pi, val))

        self.State.t1 = limitAngle(theta1)
        self.State.t2 = limitAngle(theta2)
        self.State.t3 = limitAngle(theta3)
        self.rerender()

    def setEndPositions(self, x, y, z):
        thetas = inverse_kinematics(x, y, z, self.State.F, self.State.E, self.State.Lf, self.State.Le)

        self.State.t1 = thetas[0]
        self.State.t2 = thetas[1]
        self.State.t3 = thetas[2]
        self.rerender()

    def setSpeeds(self, w1, w2, w3):
        self.State.w1 = w1
        self.State.w2 = w2
        self.State.w3 = w3
        self.rerender()

    def setAccelerations(self, e1, e2, e3):
        self.State.e1 = e1
        self.State.e2 = e2
        self.State.e3 = e3
        self.rerender()

    def mainloop(self):
        self.engine.mainloop()


if __name__ == '__main__':
    UI().mainloop()
