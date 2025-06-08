from math import cos, sin, pi

POINTS_CLOUD_FILENAME = 'plots_out/working_zone_point_cloud.pts'


class Point:
    x = None
    y = None
    z = None

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def toArray(self):
        return [self.x, self.y, self.z]

    def __str__(self):
        return f'{self.x} {self.y} {self.z}'


def save_points_cloud_to_file(points):
    with open(POINTS_CLOUD_FILENAME, 'w', encoding="utf-8") as f:
        f.write(f'{len(points)}\n')
        f.writelines(list(map(lambda p: str(p) + '\n', points)))


def linspace(from_val, to_val, step):
    res = []
    i = from_val
    while i < to_val:
        res.append(i)
        i += step
    return res


def rotate2D(x, y, degrees, x0=0, y0=0):
    rad = degrees / 180 * pi
    adjusted_x = (x - x0)
    adjusted_y = (y - y0)
    cos_rad = cos(rad)
    sin_rad = sin(rad)
    qx = x0 + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = y0 + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return qx, qy
