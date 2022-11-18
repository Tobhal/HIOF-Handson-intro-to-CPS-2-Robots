import time
import requests
import operator

from threading import Lock
from dataclasses import dataclass
from enum import Enum
from typing import Optional


@dataclass
class Vec2:
    x: float
    y: float

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def to_pose(self):
        return Pose(self.x, self.y, 0.0)


@dataclass
class Vec3(Vec2):
    z: float

    def __add__(self, other):
        if type(other) is Vec2:
            return Vec3(self.x + other.x, self.y + other.y, self.z)
        else:
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def to_pose(self):
        return Pose(self.x, self.y, self.z)


@dataclass
class Pose(Vec3):
    rx: float
    ry: float
    rz: float

    def __init__(self, x: float, y: float, z: float, rx=0.0, ry=3.14, rz=0.0):
        super().__init__(x, y, z)

        self.rx = rx
        self.ry = ry
        self.rz = rz

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
    NONE = ""
    R1 = "rob1"
    R2 = "rob2"

    @staticmethod
    def flip(state):
        return RobotPickUp.R2 if state == RobotPickUp.R1 else RobotPickUp.R1


class Object(Enum):
    CUBE = 0
    CYLINDER = 1


class Status(Enum):
    NOT_READY = 0
    READY = 1
    MOVING = 2
    ERROR = 3


if __name__ == '__main__':
    pass
