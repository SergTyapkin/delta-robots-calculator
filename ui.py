from dataclasses import dataclass
from math import *
from vendor.tiny_3d_engine.scene3d import Scene3D
from vendor.tiny_3d_engine.engine import Engine3D
import numpy as np

from src.kinematics import straight_kinematics

scene = Scene3D("World")


@dataclass
class State:
    F = 1
    E = 0.3
    Lf = 0.7
    Le = 1.2

    t1 = 0
    t2 = 0
    t3 = 0


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


def addTriangle(name: str, p1, p2, p3, color: str):
    scene.update(
        name=name,
        points=[p1, p2, p3],
        conn=[[0, 1, 2]],
        color=color,
    )


def addLine(name: str, p1, p2, color: str, width=1):
    scene.update(
        name=name,
        points=[p1, p2],
        conn=[[0, 1]],
        color=color,
        width=width,
    )


# ---- AXIS
def createAxis():
    addLine(
        "Z-axis",
        [0, 0, 0.5],
        [0, 0, -1],
        "#0000ff",
    )
    addLine(
        "Y-axis",
        [0, -0.5, 0],
        [0, 1, 0],
        "#00ff00",
    )
    addLine(
        "X-axis",
        [-0.5, 0, 0],
        [1, 0, 0],
        "#ff0000",
    )


# ---- GEOMETRY
def createDeltaRobot(F, E, Lf, Le, theta1, theta2, theta3):
    addTriangle(
        "TopBase",
        [0, triangleFromHeightPointToVertex(F), 0],
        rotate3D([0, triangleFromHeightPointToVertex(F), 0], rz=degToRad(120)),
        rotate3D([0, triangleFromHeightPointToVertex(F), 0], rz=degToRad(-120)),
        "#ff0000",
    )
    ePos = np.array(straight_kinematics(theta1, theta2, theta3, F, E, Lf, Le))

    LFstart = np.array([0, -triangleFromHeightPointToSide(F), 0])
    LFend = LFstart + rotate3D([0, -Lf, 0], rx=-theta1)
    LINE_WIDTHS = 3
    addLine(
        "Lf1",
        LFstart,
        LFend,
        "#ffff00",
        LINE_WIDTHS,
    )
    addLine(
        "Le1",
        LFend,
        ePos + [0, -triangleFromHeightPointToSide(E), 0],
        "#ffff00",
        LINE_WIDTHS,
    )

    LFstart = np.array([0, -triangleFromHeightPointToSide(F), 0])
    LFendRotX = LFstart + rotate3D([0, -Lf, 0], rx=-theta3)
    LFend = rotate3D(LFendRotX, rz=degToRad(120))
    addLine(
        "Lf2",
        rotate3D([0, -triangleFromHeightPointToSide(F), 0], rz=degToRad(120)),
        LFend,
        "#ffff00",
        LINE_WIDTHS,
    )
    addLine(
        "Le2",
        LFend,
        ePos + rotate3D([0, -triangleFromHeightPointToSide(E), 0], rz=degToRad(120)),
        "#ffff00",
        LINE_WIDTHS,
    )

    LFstart = np.array([0, -triangleFromHeightPointToSide(F), 0])
    LFendRotX = LFstart + rotate3D([0, -Lf, 0], rx=-theta2)
    LFend = rotate3D(LFendRotX, rz=degToRad(-120))
    addLine(
        "Lf3",
        rotate3D([0, -triangleFromHeightPointToSide(F), 0], rz=degToRad(-120)),
        LFend,
        "#ffff00",
        LINE_WIDTHS,
    )
    addLine(
        "Le3",
        LFend,
        ePos + rotate3D([0, -triangleFromHeightPointToSide(E), 0], rz=degToRad(-120)),
        "#ffff00",
        LINE_WIDTHS,
    )

    addTriangle(
        "BottomBase",
        ePos + [0, triangleFromHeightPointToVertex(E), 0],
        ePos + rotate3D([0, triangleFromHeightPointToVertex(E), 0], rz=degToRad(120)),
        ePos + rotate3D([0, triangleFromHeightPointToVertex(E), 0], rz=degToRad(-120)),
        "#0000ff",
    )


def updateScene():
    scene.clear()
    createAxis()
    createDeltaRobot(State.F, State.E, State.Lf, State.Le, State.t1, State.t2, State.t3)


def redraw():
    engine.clear()
    updateScene()
    engine.render()


def setAngles(theta1, theta2, theta3):
    State.t1 = theta1
    State.t2 = theta2
    State.t3 = theta3
    redraw()


engine = None
if __name__ == '__main__':
    State.t1 = degToRad(-25)
    State.t2 = degToRad(0)
    State.t3 = degToRad(90)

    ANGLE_DELTA = degToRad(5)

    updateScene()
    engine = Engine3D(scene, background="#1e1f22", shading="radial", keyHandlers={
        "q": lambda _: setAngles(State.t1 + ANGLE_DELTA, State.t2, State.t3),
        "1": lambda _: setAngles(State.t1 - ANGLE_DELTA, State.t2, State.t3),
        "w": lambda _: setAngles(State.t1, State.t2 + ANGLE_DELTA, State.t3),
        "2": lambda _: setAngles(State.t1, State.t2 - ANGLE_DELTA, State.t3),
        "e": lambda _: setAngles(State.t1, State.t2, State.t3 + ANGLE_DELTA),
        "3": lambda _: setAngles(State.t1, State.t2, State.t3 - ANGLE_DELTA),
    })
    # engine.rotate("x", 225)
    engine.render()
    engine.mainloop()
