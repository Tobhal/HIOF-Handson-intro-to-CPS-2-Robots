import time
import urx
import requests

from dataclasses import dataclass
from typing import List

@dataclass
class Vec2:
    x: float
    y: float

@dataclass
class Vec3:
    x: float
    y: float
    z: float

@dataclass
class Pose:
    x: float
    y: float
    z: float
    rx = 0.0
    ry = 3.14
    rz = 0

    def toTuple(self) -> List:
        return (self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def __add__(self, other):
        if other is type(Vec2):
            self.x + other.x
            self.y + other.y
        elif other is type(Vec3):
            self.x + other.x
            self.y + other.y
            self.z + other.z
        elif other is type(Pose):
            self.__add__(other)

class Convenor:
    def __init__(self, rob: urx.Robot) -> None:
        self.rob = rob

    def getDistance(self, camera: int) -> float:
        #change port[n] to change sensor. 1 is closest to the door, 4 is furthest away fromthe door
        r = requests.post('http://10.1.1.9', json={"code":"request","cid":1,"adr":"/getdatamulti","data":{"datatosend":[f"/iolinkmaster/port[{camera}]/iolinkdevice/pdin"]}})
        res = r.json()
        res1 = res['data']
        data = str(res1)
        print(res)
        if data[53] == "2":
            d = data[68]+data[69]
            return int(d,16)
        else:
            print("out of range")
            return None

    def start(self):
        #start coveyor
        self.rob.set_digital_out(5, 1)
        #allow digital out 5 to stay active for 0.1s
        time.sleep(0.1)
        #set digital out back to 0
        self.rob.set_digital_out(5, 0)
        #conveyor started

    def stop(self):
        #stop conveyor
        self.rob.set_digital_out(7, 1)
        #allow digital out 7 to stay active for 0.1s
        time.sleep(0.1)
        #set digital out back to 0
        self.rob.set_digital_out(7, 0)
        #conveyor stopped

    def reverse(self):
        #start coveyor in reverse direction
        self.rob.set_digital_out(6, 1)
        #allow digital out 6 to stay active for 0.1s
        time.sleep(0.1)
        #set digital out back to 0
        self.rob.set_digital_out(6, 0)
        #conveyor started in reverse direction

    def setSpeed(self, voltage):
        #sets analog out to voltage instead of current
        self.rob.send_program("set_analog_outputdomain(1, 1)")
        #sets analog out 1 to desired voltage. 0.012 is the slowest speed.
        self.rob.set_analog_out(1, voltage)