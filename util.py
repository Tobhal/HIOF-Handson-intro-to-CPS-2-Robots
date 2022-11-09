import time
import urx
import requests
import urllib.request
import operator

from threading import Lock
from Gripper import *
from dataclasses import dataclass
from typing import List
from enum import Enum

@dataclass
class Vec2:
    x: float
    y: float

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def toPose(self):
        return Pose(self.x, self.y, 0.0)

@dataclass
class Vec3(Vec2):
    z: float

    def __add__(self, other):
        if type(other) is Vec2:        
            return Vec3(self.x + other.x, self.y + other.y, self.z)
        else:
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def toPose(self):
        return Pose(self.x, self.y, self.z)

@dataclass
class Pose(Vec3):
    rx: float
    ry: float
    rz: float

    def __init__(self, x: float, y: float, z: float, rx = 0.0, ry = 3.14, rz = 0.0):
        super().__init__(x, y, z)

        self.rx = rx
        self.ry = ry
        self.rz = rz

    def toTuple(self) -> List:
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
    def __init__(self, ip: str):
        self.ip = ip
    
    def locateObject(self) -> Vec2:
        # check for response
        page = urllib.request.urlopen(f'http://{self.ip}/CmdChannel?TRIG')
        time.sleep(2)

        # Get coords
        page = urllib.request.urlopen(f'http://{self.ip}/CmdChannel?gRES')

        #reads output from camera
        coords = page.read().decode('utf-8')

        print(coords)

        #splits output
        # objectLocated = int(coords.split()[2])

        print(coords.split()[2])
        _, _, _, x, y = coords.split()[2].split(',')

        pos = Vec2((float(x) + 25) / 1000, (float(y) - 385) / 1000)

        time.sleep(3)

        return pos

class RobotPickUp(Enum):
    NONE = ""
    R1 = "rob1"
    R2 = "rob2"

    @staticmethod
    def flip(state) -> str:
        return RobotPickUp.R2 if state == RobotPickUp.R1 else RobotPickUp.R1

class Object(Enum):
    CUBE = 0
    CYLINDER = 1

class Status(Enum):
    NOT_READY = 0
    READY = 1
    MOVING = 2

class Robot(urx.Robot):
    a = 0.5
    v = 0.8

    status = Status.NOT_READY

    def __init__(self, host: str, name: str, cords: dict, lock: Lock, use_rt=False, use_simulation=False):
        super().__init__(host, use_rt, use_simulation)
        self.name = name
        self.cords = cords
        self.lock = lock

        #activates gripper. only needed once per power cycle
        self.send_program(rq_activate())
        time.sleep(2.5)
        #sets speed of gripper to max
        self.send_program(rq_set_speed(250))
        time.sleep(0.1)
        #sets force of gripper to a low value
        self.send_program(rq_set_force(10))
        time.sleep(0.1)
        #sets robot tcp, the distance from robot flange to gripper tips. 
        self.set_tcp((0,0,0.16,0,0,0))

        self.status = Status.READY

    def pickObject(self, location: Pose, object = Object.CUBE):
        """
        Pick up a object at a location.
        Default to `CUBE` object
        """
        with self.lock:
            self.send_program(rq_open())
        time.sleep(0.1)

        print(f'{self.name}: move above pick up = {location}')
        self.move(location + self.cords[object]['over'])

        self.move(location + self.cords[object]['at'])
        
        with self.lock:
            self.send_program(rq_close())
        time.sleep(0.6)

        self.move(location + self.cords[object]['over'])

    def placeObject(self, location: Pose, object = Object.CUBE):
        """
        Place a object at a location.
        Default to `CUBE` object
        """
        print(f'{self.name}: move above place = {location}')
        self.move(location + self.cords[object]['over'])

        self.move(location + self.cords[object]['at'])

        with self.lock:
            self.send_program(rq_open())

        time.sleep(1)

        self.move(location + self.cords[object]['over'])

    def centerObject(self, location: Pose, object = Object.CUBE):
        """
        Center the object in the TCP. Grabs the object at one location, TCP rotates 90 deg and grabs again.
        Uses the location for the centring
        Default to `CUBE` object
        """
        centerLocation = Pose(location.x, location.y, location.z)

        with self.lock:
            self.send_program(rq_open())

        print(f'{self.name}: center object at = {location}')        
        centerLocation.rx = 0.0
        centerLocation.ry = 3.14

        self.move(centerLocation + self.cords[object]['over'])
        self.move(centerLocation + self.cords[object]['at'])

        with self.lock:
            self.send_program(rq_close())
        time.sleep(0.5)

        with self.lock:
            self.send_program(rq_open())
        time.sleep(0.5)

        self.move(centerLocation + self.cords[object]['over'])

        centerLocation.rx = 2.2
        centerLocation.ry = 2.2
        self.move(centerLocation + self.cords[object]['over'])

        self.move(centerLocation + self.cords[object]['at'])
        with self.lock:
            self.send_program(rq_close())
        time.sleep(0.5)
        with self.lock:
            self.send_program(rq_open())
        time.sleep(0.5)

        self.move(centerLocation + self.cords[object]['over'])
    

    def move(self, location: Pose, moveWait = True):
        """
        Function for moving robot using moveJ. 
        Acquires the thread lock for just the movement of the robot
        """
        if type(location) == Vec2:
            location = location.toPose()
        elif type(location) == Vec3:
            location = location.toPose()
        
        with self.lock:
            #moves robot
            self.movex("movej", location.toTuple(), acc=self.a, vel=self.a, wait=moveWait, relative=False, threshold=None)

        if moveWait:
            time.sleep(0.1)

    def moveObject(self, fromPos: Vec3, toPos: Vec3, object = Object.CUBE, stopAtIdle = True):
        """
        Move object from position to position. Leaves the robot above the object.
        Default to `CUBE` object
        """
        self.pickObject(fromPos, object)
        
        if stopAtIdle:
            self.move(self.cords['idlePose'])

        self.placeObject(toPos, object)

    def moveObjectToConveyor(self, pickPos: Vec2, object = Object.CUBE):
        """
        Moves object form `pickPos` to the `conveyor` position. 
        Default to `CUBE` object
        """
        self.moveObject(pickPos, self.cords['conveyor'] + Vec3(0.0, 0.0, 0.001), object)

    def moveObjectFromConveyor(self, object = Object.CUBE): 
        """
        Move object from conveyor to table
        """
        conv = self.cords['conveyor']
        conv.rx = 0.0
        conv.ry = 3.14

        self.moveObject(conv, self.cords['object']['place'], object)

        conv.rx = 2.2
        conv.ry = 2.2

class Conveyor:
    """
    Static class for handling the conveyor.
    """
    mainSpeed = 0.13
    stopSpeed = 0.025

    waitTime = 1

    waitAfterDetectLeft = 2
    waitAfterDetectRight = 2

    distToWall = 50

    lock = None

    status = Status.READY

    @staticmethod
    def getDistance(sensor: int) -> float:
        """
        Change port[n] to change sensor. 1 is closest to the door, 4 is furthest away from the door
        """
        r = requests.post('http://10.1.1.9', json={"code":"request","cid":1,"adr":"/getdatamulti","data":{"datatosend":[f"/iolinkmaster/port[{sensor}]/iolinkdevice/pdin"]}})
        res = r.json()
        res1 = res['data']
        data = str(res1)

        if data[53] == "2":
            d = data[68]+data[69]
            return int(d,16)
        else:
            print("out of range")
            return None

    @staticmethod
    def start_right(rob: Robot, lock: Lock):
        """
        Moves the conveyor to the right
        """
        with lock:
            rob.set_digital_out(5, 1)
            #allow digital out 5 to stay active for 0.1s
            time.sleep(0.1)
            #set digital out back to 0
            rob.set_digital_out(5, 0)

    @staticmethod
    def start_left(rob: Robot, lock: Lock):
        """
        Moves the conveyor to the left
        """
        with lock:
            rob.set_digital_out(6, 1)
            #allow digital out 6 to stay active for 0.1s
            time.sleep(0.1)
            #set digital out back to 0
            rob.set_digital_out(6, 0)

    @staticmethod
    def stop(rob: Robot, lock: Lock):
        """
        Stops the conveyor.
        """
        with lock:
            rob.set_digital_out(7, 1)
            #allow digital out 7 to stay active for 0.1s
            time.sleep(0.1)
            #set digital out back to 0
            rob.set_digital_out(7, 0)

    @staticmethod
    def setSpeed(rob: Robot, lock: Lock, voltage: float):
        """
        Sets the speed of the conveyor. The speed is given in voltage
        """
        #sets analog out to voltage instead of current
        with lock:
            rob.send_program("set_analog_outputdomain(1, 1)")
            #sets analog out 1 to desired voltage. 0.012 is the slowest speed.
            rob.set_analog_out(1, voltage)

    @staticmethod
    def blockForDetectObject(sensor: int, compare = operator.gt):
        """
        Blocks the thread while waiting for something to move past the given sensor.
        operator.gt = >
        operator.lt = <
        """
        while compare(Conveyor.getDistance(sensor), Conveyor.distToWall):
            pass

if __name__ == '__main__':
    pass