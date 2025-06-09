"""
Field of Vision
---------------

Here are the rotation and projection for a point.

In this world, points are in a non-dimensional space,
relative to the point of view.
Coordinates changes, and can switch, upon the dialog with the user.

"""
import numpy as np
from .scene3d import comp_min_max

__all__ = ["ViewField"]

FLOAT = np.float32


class ViewField:
    def __init__(self, width: int, height: int):
        """Initiate the view field

        :param width: x_size of canvas
        :param height: y_size of canvas
                """
        self.zeros = (0.5 * width, 0.5 * height)
        self.init_scale = 0.2 * (width + height)
        # Link sclae with parent value

        self.center = None
        self.size = None
        self._initialPts = None
        self.pts = None
        self.min = None
        self.max = None
        self.revertToInitialTransform()

    def update(self, points: np.array):
        """Update the points inside the viewfield

        :param points: numpy array of size (n,3)
        """
        (self.min, self.max) = comp_min_max(points)
        self.center = 0.5 * (self.min + self.max)
        self.size = 0.5 * np.amax(self.max - self.min, axis=0)
        self.center = 0.5 * (self.min + self.max).astype(FLOAT)
        self._initialPts = np.array((points - self.center[np.newaxis, :]).astype(FLOAT) / self.size)
        self._recalculatePoints()

    def revertToInitialTransform(self):
        self.rotatingMatrix = np.matrix([
            [1.0, .0, .0],
            [.0, 1.0, .0],
            [.0, .0, 1.0]
        ])
        self.translationVector = np.array([.0, .0, .0])

    def flatten(self, distance: float, scale: float):
        """calculate 2D coordinates from 3D point"""

        proj = np.zeros((self.pts.shape[0], 2), dtype=FLOAT)

        proj[:, 0] = self.zeros[0] - scale * (
                (self.pts[:, 0] * distance)
                / (self.pts[:, 2] + distance)
        )

        proj[:, 1] = self.zeros[1] - scale * (
                (self.pts[:, 1] * distance)
                / (self.pts[:, 2] + distance)
        )
        return proj.astype(int)

    def rotate(self, axis: str, angle: float):
        """rotate point around axis"""
        angle = angle / 180 * np.pi
        if axis == 'z':
            # rotate aroud Z axis
            self.rotatingMatrix *= np.matrix([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1]
            ])
        elif axis == 'x':
            # rotate around X axis
            self.rotatingMatrix *= np.matrix([
                [1, 0, 0],
                [0, np.cos(angle), -np.sin(angle)],
                [0, np.sin(angle), np.cos(angle)]
            ])
        elif axis == 'y':
            # rotate around Y axis
            self.rotatingMatrix *= np.matrix([
                [np.cos(angle), 0, -np.sin(angle)],
                [0, 1, 0],
                [np.sin(angle), 0, np.cos(angle)]
            ])
        else:
            raise ValueError('not a valid axis')
        self._recalculatePoints()

    def translate(self, axis: str, amount: float):
        """tranlate point by distance"""
        if axis == 'z':
            self.translationVector[2] += amount
        elif axis == 'x':
            self.translationVector[0] += amount
        elif axis == 'y':
            self.translationVector[1] += amount
        else:
            raise ValueError('not a valid axis')

    def _recalculatePoints(self):
        def rotate3D(p, matrix):
            return np.array((np.array(p) * matrix).tolist()[0])
        self.pts = np.array(list(map(lambda p: rotate3D(p, self.rotatingMatrix), self._initialPts)))
        self.pts = np.array(list(map(lambda p: p + self.translationVector, self.pts)))
