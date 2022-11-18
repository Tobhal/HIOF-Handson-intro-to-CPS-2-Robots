import time
import urx
import requests
import urllib.request
import operator

from threading import Lock
from Gripper import *
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


class Camera:
    switchCounter = 0
    witchObject = 0
    objectLocated = 0

    def __init__(self, ip: str):
        self.ip = ip

    def locate_object(self) -> Vec2:
        # check for response
        page = urllib.request.urlopen(f'http://{self.ip}/CmdChannel?TRIG')
        time.sleep(2)

        # Get coords
        page = urllib.request.urlopen(f'http://{self.ip}/CmdChannel?gRES')

        # reads output from camera
        coords = page.read().decode('utf-8')

        print(coords)

        # splits output
        print(coords.split()[2])
        _, self.witchObject, self.objectLocated, x, y = coords.split()[2].split(',')

        pos = Vec2((float(x) + 25) / 1000, (float(y) - 385) / 1000)

        time.sleep(3)

        return pos

    def switch_object(self):
        self.switchCounter += 1
        if self.witchObject == 0:
            urllib.request.urlopen('http://10.1.1.8/CmdChannel?sINT_1_1')
            time.sleep(3)
        if self.witchObject == 1:
            urllib.request.urlopen('http://10.1.1.8/CmdChannel?sINT_1_0')
            time.sleep(3)
        time.sleep(1)
        print(f"Camera {self.ip}: object switched")


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


class Robot(urx.Robot):
    a = 0.5
    v = 0.8

    status = Status.NOT_READY

    def __init__(self, host: str, name: str, cords: dict, use_rt=False, use_simulation=False):
        super().__init__(host, use_rt, use_simulation)
        self.name = name
        self.cords = cords
        self.lock = Lock()

        # activates gripper. only needed once per power cycle
        self.send_program(rq_activate())
        time.sleep(2.5)
        # sets speed of gripper to max
        self.send_program(rq_set_speed(250))
        time.sleep(0.1)
        # sets force of gripper to a low value
        self.send_program(rq_set_force(10))
        time.sleep(0.1)
        # sets robot tcp, the distance from robot flange to gripper tips.
        self.set_tcp((0, 0, 0.16, 0, 0, 0))

        self.status = Status.READY

    def pick_object(self, location: Pose, object=Object.CUBE):
        """
        Pick up a object at a location.
        Default to `CUBE` object
        """
        with self.lock:
            self.send_program(rq_open())
        # time.sleep(0.1)

        print(f'{self.name}: move above pick up = {location}')
        self.move(location + self.cords[object]['over'])

        self.move(location + self.cords[object]['at'])

        with self.lock:
            self.send_program(rq_close())
        time.sleep(0.6)

        self.move(location + self.cords[object]['over'])

    def place_object(self, location: Pose, object=Object.CUBE):
        """
        Place a object at a location.
        Default to `CUBE` object
        """
        print(f'{self.name}: move above place = {location}')
        self.move(location + self.cords[object]['over'])

        self.move(location + self.cords[object]['at'])

        with self.lock:
            self.send_program(rq_open())
        time.sleep(0.1)

        self.move(location + self.cords[object]['over'])

    def center_object(self, location: Pose, object=Object.CUBE):
        """
        Center the object in the TCP. Grabs the object at one location, TCP rotates 90 deg and grabs again.
        Uses the location for the centring
        Default to `CUBE` object
        """
        center_location = Pose(location.x, location.y, location.z)

        with self.lock:
            self.send_program(rq_open())

        print(f'{self.name}: center object at = {location}')
        center_location.rx = 0.0
        center_location.ry = 3.14

        def open_close():
            self.move(center_location + self.cords[object]['over'])
            self.move(center_location + self.cords[object]['at'])

            with self.lock:
                self.send_program(rq_close())
            time.sleep(0.5)

            with self.lock:
                self.send_program(rq_open())
            time.sleep(0.5)

            self.move(center_location + self.cords[object]['over'])

        open_close()

        center_location.rx = 2.2
        center_location.ry = 2.2

        open_close()

    def move(self, location: Pose, move_wait=True):
        """
        Function for moving robot using moveJ. 
        Acquires the thread lock for just the movement of the robot
        """
        self.status = Status.MOVING

        if type(location) == Vec2:
            location = location.to_pose()
        elif type(location) == Vec3:
            location = location.to_pose()

        with self.lock:
            # moves robot
            self.movex("movej", location.to_tuple(), acc=self.a, vel=self.a, wait=move_wait, relative=False,
                       threshold=None)

        if move_wait:
            time.sleep(0.1)

        self.status = Status.READY

    def move_object(self, from_pos: Vec3, to_pos: Vec3, object=Object.CUBE, stop_at_idle=True, wait_at_idle=False):
        """
        Move object from position to position. Leaves the robot above the object.
        Default to `CUBE` object
        """
        self.pick_object(from_pos.to_pose(), object)

        if stop_at_idle:
            self.move(self.cords['idlePose'], wait_at_idle)

        self.place_object(to_pos.to_pose(), object)

    def move_object_to_conveyor(self, pick_pos: Vec3, object=Object.CUBE):
        """
        Moves object form `pickPos` to the `conveyor` position. 
        Default to `CUBE` object
        """
        self.move_object(pick_pos, self.cords['conveyor'] + Vec3(0.0, 0.0, 0.001), object)

    def move_object_from_conveyor(self, object=Object.CUBE):
        """
        Move object from conveyor to table
        """
        conv = self.cords['conveyor']
        conv.rx = 0.0
        conv.ry = 3.14

        self.move_object(conv, self.cords['object']['place'], object)

        conv.rx = 2.2
        conv.ry = 2.2


class Conveyor:
    """
    Static class for handling the conveyor.
    """
    main_speed = 0.15
    """Main speed of the conveyor"""
    stop_speed = 0.025
    """Speed before detection from sensor 1 and 4"""

    wait_time = 1
    """Time to before starting conveyor, normally after robot have placed object"""

    wait_after_detect_left = 3
    """Wait after sensor 3 has detected the object"""

    wait_after_detect_right = 3.3
    """Wait after sensor 2 has detected the object"""

    dist_to_wall = 50
    robot: Robot = None
    lock: Lock = None
    status = Status.READY

    @staticmethod
    def get_distance(sensor: int) -> Optional[float]:
        """
        Change port[n] to change sensor. 1 is closest to the door, 4 is the furthest away from the door
        """
        r = requests.post('http://10.1.1.9', json={"code": "request", "cid": 1, "adr": "/getdatamulti", "data": {
            "datatosend": [f"/iolinkmaster/port[{sensor}]/iolinkdevice/pdin"]}})
        res = r.json()
        res1 = res['data']
        data = str(res1)

        if data[53] == "2":
            d = data[68] + data[69]
            return int(d, 16)
        else:
            print("out of range")
            return None

    @staticmethod
    def start_right():
        """
        Moves the conveyor to the right
        """
        with Conveyor.lock:
            Conveyor.robot.set_digital_out(5, 1)
            # allow digital out 5 to stay active for 0.1s
            time.sleep(0.1)
            # set digital out back to 0
            Conveyor.robot.set_digital_out(5, 0)

    @staticmethod
    def start_left():
        """
        Moves the conveyor to the left
        """
        with Conveyor.lock:
            Conveyor.robot.set_digital_out(6, 1)
            # allow digital out 6 to stay active for 0.1s
            time.sleep(0.1)
            # set digital out back to 0
            Conveyor.robot.set_digital_out(6, 0)

    @staticmethod
    def stop():
        """
        Stops the conveyor.
        """
        with Conveyor.lock:
            Conveyor.robot.set_digital_out(7, 1)
            # allow digital out 7 to stay active for 0.1s
            time.sleep(0.1)
            # set digital out back to 0
            Conveyor.robot.set_digital_out(7, 0)

    @staticmethod
    def set_speed(voltage: float):
        """
        Sets the speed of the conveyor. The speed is given in voltage
        """
        # sets analog out to voltage instead of current
        with Conveyor.lock:
            Conveyor.robot.send_program("set_analog_outputdomain(1, 1)")
            # sets analog out 1 to desired voltage. 0.012 is the slowest speed.
            Conveyor.robot.set_analog_out(1, voltage)

    @staticmethod
    def block_for_detect_object(sensor: int, compare=operator.gt):
        """
        Blocks the thread while waiting for something to move past the given sensor.
        operator.gt = >
        operator.lt = <
        """
        while compare(Conveyor.get_distance(sensor), Conveyor.dist_to_wall):
            pass


if __name__ == '__main__':
    pass
