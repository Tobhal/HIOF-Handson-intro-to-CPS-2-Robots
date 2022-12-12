from dataclasses import dataclass
from enum import Enum


@dataclass
class Vec2:
    x: float
    y: float

    def to_tuple(self) -> tuple[float, float]:
        return self.x, self.y

    def to_vec2(self):
        return Vec2(self.x, self.y)

    def to_vec3(self):
        return Vec3(self.x, self.y, 0.0)

    def to_pose(self):
        return Pose(self.x, self.y, 0.0)

    def __add__(self, other):
        if type(other) is Vec2:
            return Vec2(self.x + other.x, self.y + other.y)
        else:
            return Vec2(self.x + other, self.y + other)

    def __sub__(self, other):
        if type(other) is Vec2:
            return Vec2(self.x - other.x, self.y - other.y)
        else:
            return Vec2(self.x - other, self.y - other)

    def __mul__(self, other):
        if type(other) is Vec2:
            return Vec2(self.x * other.x, self.y * other.y)
        else:
            return Vec2(self.x * other, self.y * other)

    def __truediv__(self, other):
        if type(other) is Vec2:
            return Vec2(self.x / other.x, self.y / other.y)
        else:
            return Vec2(self.x / other, self.y / other)


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def to_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z

    def to_vec2(self):
        return Vec2(self.x, self.y)

    def to_pose(self):
        return Pose(self.x, self.y, self.z)

    def __add__(self, other):
        if type(other) is Vec2:
            return Vec3(self.x + other.x, self.y + other.y, self.z)
        elif type(other) is Vec3:
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            return Vec3(self.x + other, self.y + other, self.z + other)

    def __sub__(self, other):
        if type(other) is Vec2:
            return Vec3(self.x - other.x, self.y - other.y, self.z)
        elif type(other) is Vec3:
            return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            return Vec3(self.x - other, self.y - other, self.z - other)

    def __mul__(self, other):
        if type(other) is Vec2:
            return Vec3(self.x * other.x, self.y * other.y, self.z)
        elif type(other) is Vec3:
            return Vec3(self.x * other.x, self.y * other.y, self.z * other.z)
        else:
            return Vec3(self.x * other, self.y * other, self.z * other)

    def __truediv__(self, other):
        if type(other) is Vec2:
            return Vec3(self.x / other.x, self.y / other.y, self.z)
        elif type(other) is Vec3:
            return Vec3(self.x / other.x, self.y / other.y, self.z / other.z)
        else:
            return Vec3(self.x / other, self.y / other, self.z / other)


@dataclass
class Pose:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

    def __init__(self, x: float, y: float, z: float, rx=0.0, ry=3.14, rz=0.0):
        self.x = x
        self.y = y
        self.z = z

        self.rx = rx
        self.ry = ry
        self.rz = rz

    def to_vec2(self):
        return Vec2(self.x, self.y)

    def to_vec3(self):
        return Vec3(self.x, self.y, self.z)

    def to_pose(self):
        return Pose(self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def to_tuple(self) -> tuple:
        return self.x, self.y, self.z, self.rx, self.ry, self.rz

    def __add__(self, other):
        if type(other) is Vec2:
            return Pose(self.x + other.x, self.y + other.y, 0.0, rx=self.rx, ry=self.ry, rz=self.rz)
        elif type(other) is Vec3:
            return Pose(self.x + other.x, self.y + other.y, self.z + other.z, rx=self.rx, ry=self.ry, rz=self.rz)
        else:
            return Pose(
                self.x + other.x,
                self.y + other.y,
                self.z + other.z,
                rx=self.rx + other.rx,
                ry=self.ry + other.ry,
                rz=self.rz + other.rz
            )


class RobotPickUp(Enum):
    NONE = "None"
    R1 = "rob1"
    R2 = "rob2"

    @staticmethod
    def flip(state):
        if type(state) == str:
            return RobotPickUp.R1 if state == 'rob2' else RobotPickUp.R2
        elif type(state) == RobotPickUp:
            return RobotPickUp.R2 if state == RobotPickUp.R1 else RobotPickUp.R1
        else:
            return RobotPickUp.NONE


class Object(Enum):
    CUBE = {
        'over': Vec3(0.0, 0.0, 0.1),
        'at': Vec3(0.0, 0.0, 0.01),
        'size': Vec3(0.05, 0.05, 0.05)
    }
    CYLINDER = {
        'over': Vec3(0.0, 0.0, 0.15),
        'at': Vec3(0.0, 0.0, 0.025),
        'size': Vec3(0.06, 0.06, 0.075)
    }

    @staticmethod
    def flip(obj):
        return Object.CUBE if obj == Object.CYLINDER else Object.CYLINDER

    def __getitem__(self, item):
        return self.value[item]


class Status(Enum):
    NOT_READY = 0
    READY = 1
    MOVING = 2
    WAIT = 3
    ERROR = 4


class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    NONE = 3


if __name__ == '__main__':
    pass
