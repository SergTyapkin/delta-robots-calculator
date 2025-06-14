import sys
from dataclasses import dataclass
from math import *
from typing import Callable

from src.acceleration_kinematics import straight_acceleration_kinematics
from src.speed_kinematics import straight_speed_kinematics, inverse_speed_kinematics
from vendor.tiny_3d_engine.scene3d import Scene3D
from vendor.tiny_3d_engine.engine import Engine3D
from threading import Thread
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
    def __init__(self, traceEndpoint=False, dark=False):
        self.traceEndpoint = False
        self.endpointTracing = []
        self.userObjects = []
        self.dark = dark
        self.engine = None

        self.scene = Scene3D("World")
        self.update()

        self.traceEndpoint = traceEndpoint

        ANGLE_DELTA = degToRad(5)
        SPEED_DELTA = 0.1
        ACCELERATION_DELTA = 0.1
        POSITION_DELTA = 0.1
        self.engine = Engine3D(self.scene, dark=dark, shading="flat", keyHandlers={
            "exclam": lambda _: self.setAngles(self.State.t1 + ANGLE_DELTA, self.State.t2, self.State.t3),
            "1": lambda _: self.setAngles(self.State.t1 - ANGLE_DELTA, self.State.t2, self.State.t3),
            "at": lambda _: self.setAngles(self.State.t1, self.State.t2 + ANGLE_DELTA, self.State.t3),
            "2": lambda _: self.setAngles(self.State.t1, self.State.t2 - ANGLE_DELTA, self.State.t3),
            "numbersign": lambda _: self.setAngles(self.State.t1, self.State.t2, self.State.t3 + ANGLE_DELTA),
            "3": lambda _: self.setAngles(self.State.t1, self.State.t2, self.State.t3 - ANGLE_DELTA),

            "Q": lambda _: self.setAngleSpeeds(self.State.w1 + SPEED_DELTA, self.State.w2, self.State.w3),
            "q": lambda _: self.setAngleSpeeds(self.State.w1 - SPEED_DELTA, self.State.w2, self.State.w3),
            "W": lambda _: self.setAngleSpeeds(self.State.w1, self.State.w2 + SPEED_DELTA, self.State.w3),
            "w": lambda _: self.setAngleSpeeds(self.State.w1, self.State.w2 - SPEED_DELTA, self.State.w3),
            "E": lambda _: self.setAngleSpeeds(self.State.w1, self.State.w2, self.State.w3 + SPEED_DELTA),
            "e": lambda _: self.setAngleSpeeds(self.State.w1, self.State.w2, self.State.w3 - SPEED_DELTA),

            "A": lambda _: self.setAngleAccelerations(self.State.e1 + ACCELERATION_DELTA, self.State.e2, self.State.e3),
            "a": lambda _: self.setAngleAccelerations(self.State.e1 - ACCELERATION_DELTA, self.State.e2, self.State.e3),
            "S": lambda _: self.setAngleAccelerations(self.State.e1, self.State.e2 + ACCELERATION_DELTA, self.State.e3),
            "s": lambda _: self.setAngleAccelerations(self.State.e1, self.State.e2 - ACCELERATION_DELTA, self.State.e3),
            "D": lambda _: self.setAngleAccelerations(self.State.e1, self.State.e2, self.State.e3 + ACCELERATION_DELTA),
            "d": lambda _: self.setAngleAccelerations(self.State.e1, self.State.e2, self.State.e3 - ACCELERATION_DELTA),

            "x": lambda _: self.setEndPosition(self.points.E.center[0] + POSITION_DELTA, self.points.E.center[1],
                                               self.points.E.center[2]),
            "X": lambda _: self.setEndPosition(self.points.E.center[0] - POSITION_DELTA, self.points.E.center[1],
                                               self.points.E.center[2]),
            "y": lambda _: self.setEndPosition(self.points.E.center[0], self.points.E.center[1] + POSITION_DELTA,
                                               self.points.E.center[2]),
            "Y": lambda _: self.setEndPosition(self.points.E.center[0], self.points.E.center[1] - POSITION_DELTA,
                                               self.points.E.center[2]),
            "z": lambda _: self.setEndPosition(self.points.E.center[0], self.points.E.center[1],
                                               self.points.E.center[2] + POSITION_DELTA),
            "Z": lambda _: self.setEndPosition(self.points.E.center[0], self.points.E.center[1],
                                               self.points.E.center[2] - POSITION_DELTA),
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

        x = 0
        y = 0
        z = 0

        vx = 0
        vy = 0
        vz = 0

        ax = 0
        ay = 0
        az = 0

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

    def addPolylineByPoints(self, name: str, points: list, color: str, width=1):
        conn = []
        for i in range(len(points)):
            if i < len(points) - 1:
                conn.append([i, i + 1])
        if len(points) <= 1:
            return
        self.scene.update(
            name=name,
            points=points,
            conn=conn,
            color=color,
        )

    def addPoint(self, name: str, point: list, color: str, width=10):
        self.scene.update(
            name=name,
            points=[point],
            conn=[[0]],
            color=color,
            width=width,
        )

    def addArrow(self, name: str, start: list, end: list, color: str, width=1):
        if not self.engine or not self.engine.scale:
            return
        start = np.array(start)
        end = np.array(end)
        ds = 10 / self.engine.scale
        dz = np.sign(end[2] - start[2]) * ds
        self.scene.update(
            name=name,
            points=[start, end, end + [-ds, 0, -dz], end + [ds, 0, -dz], end + [0, -ds, -dz], end + [0, ds, -dz]],
            conn=[[0, 1], [1, 2], [1, 3], [1, 4], [1, 5]],
            color=color,
            width=width,
        )

    # ---- AXIS
    def addAxes(self):
        # self.scene.add_axes('zero')
        # Create my own axes
        axesLength = 1.5
        self.addLineByPoints(
            "Z-axis",
            [0, 0, 1],
            [0, 0, -axesLength],
            "#0000ff",
        )
        self.addLineByPoints(
            "Y-axis",
            [0, -axesLength, 0],
            [0, axesLength, 0],
            "#00ff00",
        )
        self.addLineByPoints(
            "X-axis",
            [-axesLength, 0, 0],
            [axesLength, 0, 0],
            "#ff0000",
        )

    def addUserObject(self, foo: Callable, args: list = []):
        self.userObjects.append([foo, args])

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
        SPEED_LENGTH_FACTOR = 0.05
        ACC_LENGTH_FACTOR = 0.05
        OMEGA_LENGTH_FACTOR = 0.05
        EPS_LENGTH_FACTOR = 0.05
        TRACING_COLOR = "#ffffff" if self.dark else "#000000"
        TRACING_WIDTH = 1

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

        self.addArrow(f"w1_{round(self.State.w1, 2)}", self.points.LF1.start,
                      self.points.LF1.start + [0, 0, -self.State.w1 * OMEGA_LENGTH_FACTOR],
                      SPEEDS_LINES_COLOR, SPEEDS_LINES_WIDTHS)
        self.addArrow(f"w2_{round(self.State.w2, 2)}", self.points.LF2.start,
                      self.points.LF2.start + [0, 0, -self.State.w2 * OMEGA_LENGTH_FACTOR],
                      SPEEDS_LINES_COLOR, SPEEDS_LINES_WIDTHS)
        self.addArrow(f"w3_{round(self.State.w3, 2)}", self.points.LF3.start,
                      self.points.LF3.start + [0, 0, -self.State.w3 * OMEGA_LENGTH_FACTOR],
                      SPEEDS_LINES_COLOR, SPEEDS_LINES_WIDTHS)

        self.addArrow(f"e1_{round(self.State.e1, 2)}", self.points.LF1.start,
                      self.points.LF1.start + [0, 0, -self.State.e1 * EPS_LENGTH_FACTOR],
                      ACCELERATIONS_LINES_COLOR, ACCELERATIONS_LINES_WIDTHS)
        self.addArrow(f"e2_{round(self.State.e2, 2)}", self.points.LF2.start,
                      self.points.LF2.start + [0, 0, -self.State.e2 * EPS_LENGTH_FACTOR],
                      ACCELERATIONS_LINES_COLOR, ACCELERATIONS_LINES_WIDTHS)
        self.addArrow(f"e3_{round(self.State.e3, 2)}", self.points.LF3.start,
                      self.points.LF3.start + [0, 0, -self.State.e3 * EPS_LENGTH_FACTOR],
                      ACCELERATIONS_LINES_COLOR, ACCELERATIONS_LINES_WIDTHS)

        speed = np.array([self.State.vx, self.State.vy, self.State.vz])
        self.addLineByPoints(f"speed_{round(getLength(speed), 2)}", self.points.E.center,
                             self.points.E.center + speed * SPEED_LENGTH_FACTOR,
                             SPEEDS_LINES_COLOR, SPEEDS_LINES_WIDTHS)
        # acceleration = np.array(straight_acceleration_kinematics(
        #     self.State.e1, self.State.e2, self.State.e3,
        #     self.State.w1, self.State.w2, self.State.w3,
        #     self.State.t1, self.State.t2, self.State.t3,
        #     self.State.F, self.State.E, self.State.Lf, self.State.Le)
        # )
        acceleration = np.array([self.State.ax, self.State.ay, self.State.az])
        self.addLineByPoints(f"acceleration_{round(getLength(acceleration), 2)}", self.points.E.center,
                             self.points.E.center + acceleration * ACC_LENGTH_FACTOR,
                             ACCELERATIONS_LINES_COLOR, ACCELERATIONS_LINES_WIDTHS)

        if self.traceEndpoint:
            self.addPolylineByPoints("tracing", self.endpointTracing, TRACING_COLOR, TRACING_WIDTH)

    def update(self):
        self.scene.clear()
        self.calculateDeltaRobotPoints(self.State.F, self.State.E, self.State.Lf, self.State.Le,
                                       self.State.t1, self.State.t2, self.State.t3)
        if self.traceEndpoint:
            self.endpointTracing.append(self.points.E.center)
        self.addDeltaRobotToScene()
        for userObject in self.userObjects:
            userObject[0](*userObject[1])
        self.addAxes()

    def rerender(self):
        self.engine.clear()
        self.update()
        self.engine.render()

    def setAngles(self, theta1, theta2, theta3, withRerender=True):
        def limitAngle(val: float):
            return max(-pi, min(pi, val))

        self.State.t1 = limitAngle(theta1)
        self.State.t2 = limitAngle(theta2)
        self.State.t3 = limitAngle(theta3)
        x, y, z = straight_kinematics(theta1, theta2, theta3, self.State.F, self.State.E, self.State.Lf, self.State.Le)
        self.State.x = x
        self.State.y = y
        self.State.z = z
        if withRerender:
            self.rerender()

    def setAngleSpeeds(self, w1, w2, w3, withRerender=True):
        self.State.w1 = w1
        self.State.w2 = w2
        self.State.w3 = w3
        vx, vy, vz = straight_speed_kinematics(self.State.w1, self.State.w2, self.State.w3, self.State.t1,
                                               self.State.t2, self.State.t3, self.State.F, self.State.E, self.State.Lf,
                                               self.State.Le)
        self.State.vx = vx
        self.State.vy = vy
        self.State.vz = vz
        if withRerender:
            self.rerender()

    def setAngleAccelerations(self, e1, e2, e3, withRerender=True):
        self.State.e1 = e1
        self.State.e2 = e2
        self.State.e3 = e3
        if withRerender:
            self.rerender()

    def setPosition(self, x, y, z, withRerender=True):
        self.State.x = x
        self.State.y = y
        self.State.z = z
        thetas = inverse_kinematics(x, y, z, self.State.F, self.State.E, self.State.Lf, self.State.Le)

        self.State.t1 = thetas[0]
        self.State.t2 = thetas[1]
        self.State.t3 = thetas[2]
        if withRerender:
            self.rerender()

    def setSpeeds(self, vx, vy, vz, withRerender=True):
        self.State.vx = vx
        self.State.vy = vy
        self.State.vz = vz
        w1, w2, w3 = inverse_speed_kinematics(self.State.vx, self.State.vy, self.State.vz, self.State.x, self.State.y,
                                              self.State.z, self.State.F, self.State.E, self.State.Lf, self.State.Le)
        self.State.w1 = w1
        self.State.w2 = w2
        self.State.w3 = w3
        if withRerender:
            self.rerender()

    def setAccelerations(self, ax, ay, az, withRerender=True):
        self.State.ax = ax
        self.State.ay = ay
        self.State.az = az
        if withRerender:
            self.rerender()

    def start(self, otherProcessFoo, *args, **kwargs):
        otherProcessThread = Thread(target=otherProcessFoo, args=args, kwargs=kwargs)
        otherProcessThread.start()

        self.engine.mainloop()

        print(f"🔄 Graceful exiting...")
        print(f"🔄 Wait for join worker thread to finish...")
        otherProcessThread.join()
        print(f"✅ Thread joined")
        sys.exit(0)


# Demo:
if __name__ == '__main__':
    UI().start(lambda: None)
