import time
from typing_extensions import Self
import urx
import requests
import urllib.request

from dataclasses import dataclass
from typing import List

@dataclass
class Vec2:
    x: float
    y: float

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def toPose(self):
        return Pose(self.x, self.y, 0.0)

@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def toPose(self):
        return Pose(self.x, self.y, self.z)

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
        if type(other) is Vec2:
            return Pose(self.x + other.x, self.y + other.y, 0.0)
        elif type(other) in [Vec3, Pose] :
            return Pose(self.x + other.x, self.y + other.y, self.z + other.z)

        return Pose(0.0, 0.0, 0.0)

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

class Camera:
    def __init__(self, ip: str) -> None:
        self.ip = ip
    
    def locateObject(self) -> Vec2:
        global objectLocated, switchCounter

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

        switchCounter = 0

        print(coords.split()[2])
        _, _, _, x, y = coords.split()[2].split(',')

        pos = Vec2((float(x) + 25) / 1000, (float(y) - 385) / 1000)

        time.sleep(3)

        return pos

if __name__ == '__main__':
    pose = Pose(1,2,3)

    print(pose + Vec3(0,0,4))