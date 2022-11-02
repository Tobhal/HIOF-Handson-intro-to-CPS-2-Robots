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

v = 0.8
a = 0.5

useCamera = False
activateGripper = False

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
def move(robot: urx.Robot, location: Pose, moveWait: bool) -> None:
    #moves robot
    robot.movex("movej", location.toTuple(), acc=a, vel=v, wait=moveWait, relative=False, threshold=None)
    if moveWait == False:
        time.sleep(0.1)

#Moves robot to coordinates set by camera
def moveObject(rob: urx.Robot, fromPos: Vec2, toPos: Vec2, fromTable: bool, toTable: bool) -> None:
    global lastPos, objectCount, overBlock, atBlock, overConveyer, atConveyer, idlePose
    objectCount+= 1

    lastPos = Vec2(fromPos.x, fromPos.y)

    if fromTable:
        overPickPos = Pose(fromPos.x, fromPos.y, overBlock)
        pickPos = Pose(fromPos.x, fromPos.y, atBlock)
    else:
        overPickPos = Pose(fromPos.x, fromPos.y, overConveyer)
        pickPos = Pose(fromPos.x, fromPos.y, atConveyer)

    if toTable:
        overPlaceObject = Pose(toPos.x, toPos.y, overBlock)
        placeObject = Pose(toPos.x, toPos.y, atBlock)
    else:
        overPlaceObject = Pose(toPos.x, toPos.y, overConveyer)
        placeObject = Pose(toPos.x, toPos.y, atConveyer)
    
    rob.send_program(rq_open())
    time.sleep(0.1)

    move(rob, overPickPos, True)
    move(rob, pickPos, True)
    
    #closes gripper
    rob.send_program(rq_close())
    
    #sleep to allow gripper to close fully before program resumes
    time.sleep(0.6)
    
    move(rob, overPickPos, True)
    # Object picked up

    move(rob, idlePose, True)

    move(rob, overPlaceObject, True)
    move(rob, placeObject, True)
    
    rob.send_program(rq_open())
    time.sleep(0.2)

    move(rob, overPlaceObject, True)

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
    global rob1
    move(rob1, idlePose, True)

    pos = Vec2(0.00773, -0.31881)

    moveObject(rob1, pos, conveyerPos, True, False)
    move(rob1, idlePose, True)
    moveObject(rob1, conveyerPos, pos, False, True)

def rob2Move():
    global rob2
    move(rob2, idlePose, True)

    pos = Vec2(0.00773, -0.31881)

    moveObject(rob2, pos, conveyerPos, True, False)
    move(rob2, idlePose, True)
    moveObject(rob2, conveyerPos, pos, False, True)


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