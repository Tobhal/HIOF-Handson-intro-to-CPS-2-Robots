import urx
import time

from Gripper import *
from util import *
from threading import Thread
            
rob1 = urx.Robot('10.1.1.6', use_rt=True)
rob2 = urx.Robot('10.1.1.5', use_rt=True)
conveyer = Convenor(rob1)
camera1 = Camera('10.1.1.8')
camera2 = Camera('10.1.1.7')

rob1Cords = {
    'conveyer': Vec3(-0.012, 0.4, -0.02),
    'idlePose': Vec3(0.25, -0.22, 0.09),
    'getObject': Vec3(0.00773, -0.31881, 0.0),
    'placeObject': Vec3(0.3, -0.17, 0.0),
}

rob2Cords = {
    'conveyer': Vec3(-0.012, 0.4, -0.02),
    'idlePose': Vec3(-0.25, -0.22, 0.09),
    'getObject': Vec3(0.00773, -0.31881, 0.0),
    'placeObject': Vec3(0.3, -0.17, 0.0),
}

v = 0.8
a = 0.5

useCamera = False
activateGripper = True

pos = Vec2(float(25/1000), float(-385/10000))
lastPos = Vec2(pos.x, pos.y)

objectLocated = 0
objectCount = 0

overBlock = 0.09
atBlock = 0.0

overConveyer = 0.04
atConveyer = -0.02

idlePose = Pose(0.25, -0.22, 0.09)
placeObject = Vec2(0.3, -0.17)
conveyerPos = Vec2(-0.012, 0.4)

#function for moving robot using moveJ
def move(rob: urx.Robot, location: Pose, moveWait: bool) -> None:
    #moves robot
    rob.movex("movej", location.toTuple(), acc=a, vel=v, wait=moveWait, relative=False, threshold=None)
    if not moveWait:
        time.sleep(0.1)

#Moves robot to coordinates set by camera
def moveObject(rob: urx.Robot, fromPos: Vec3, toPos: Vec3, name: str) -> None:
    global lastPos, objectCount, overConveyer, atConveyer, rob1Cords, rob2Cords
    objectCount+= 1

    if name == 'rob1':
        idlePose = rob1Cords['idlePose']
    elif name == 'rob2':
        idlePose = rob2Cords['idlePose']

    idlePose = Pose(idlePose.x, idlePose.y, idlePose.z)

    overBlock = Vec3(0.0, 0.0, 0.09)

    rob.send_program(rq_open())
    time.sleep(0.1)

    fromPos = Pose(fromPos.x, fromPos.y, fromPos.z)
    toPos = Pose(toPos.x, toPos.y, toPos.z)

    print(f'{name}: move above pick up = {fromPos + overBlock}')
    move(rob, fromPos + overBlock, True)

    print(f'{name}: move to pick up = {fromPos}')
    move(rob, fromPos, True)
    
    #closes gripper
    rob.send_program(rq_close())
    
    #sleep to allow gripper to close fully before program resumes
    time.sleep(0.6)
    
    print(f'{name}: move above pick up = {fromPos + overBlock}')
    move(rob, fromPos + overBlock, True)
    # Object picked up

    print(f'{name}: move to idle pos = {idlePose}')
    move(rob, idlePose, True)

    print(f'{name}: move above place = {toPos + overBlock}')
    move(rob, toPos + overBlock, True)

    print(f'{name}: move to place = {toPos}')
    move(rob, toPos, True)
    
    rob.send_program(rq_open())
    time.sleep(0.2)

    print(f'{name}: move above place = {toPos + overBlock}')
    move(rob, toPos + overBlock, True)

def initRobot(rob: urx.Robot):
    #activates gripper. only needed once per power cycle
    rob.send_program(rq_activate())
    time.sleep(2.5)
    #sets speed of gripper to max
    rob.send_program(rq_set_speed(250))
    time.sleep(0.1)
    #sets force of gripper to a low value
    rob.send_program(rq_set_force(10))
    time.sleep(0.1)
    #sets robot tcp, the distance from robot flange to gripper tips. 
    rob.set_tcp((0,0,0.16,0,0,0))

def rob1Move():
    global rob1, camera1, rob1Cords
    name = 'rob1'

    print(f'{name}: move to idle pos = {idlePose}')
    move(rob1, rob1Cords['idlePose'].toPose(), True)

    moveObject(rob1, rob1Cords['getObject'], rob1Cords['conveyer'], name)
    
    print(f'{name}: move to idle pos = {idlePose}')
    move(rob1, rob1Cords['idlePose'].toPose(), True)

    rob1Cords['getObject'].z = 0.002 # temp fix

    moveObject(rob1, rob1Cords['conveyer'], rob1Cords['getObject'], name)

    move(rob1, rob1Cords['idlePose'].toPose(), True)

def rob2Move():
    global rob2, camera2, rob2Cords
    name = 'rob2'

    print(f'{name}: move to idle pos = {idlePose}')
    move(rob2, rob2Cords['idlePose'].toPose(), True)

    # objectPos = camera2.locateObject()

    # objectPos = Vec3(objectPos.x, objectPos.y, 0.0)

    moveObject(rob2, rob2Cords['getObject'], rob2Cords['conveyer'], name)
    
    print(f'{name}: move to idle pos = {idlePose}')
    move(rob2, rob2Cords['idlePose'].toPose(), True)
    
    moveObject(rob2, rob2Cords['conveyer'], rob2Cords['getObject'], name)

    move(rob2, rob2Cords['idlePose'].toPose(), True)

def emptyFunc():
    pass

if __name__ == '__main__':
    if activateGripper:    
        initRobot(rob1)
        initRobot(rob2)

    rob1Thread = Thread(target=rob1Move)
    rob2Thread = Thread(target=rob2Move)

    rob1Thread.start()
    rob2Thread.start()

    rob1Thread.join()
    rob2Thread.join()

    rob1.close()
    rob2.close()