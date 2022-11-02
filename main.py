import urx
import time

from Gripper import *
from util import *
            
rob1 = urx.Robot('10.1.1.6', use_rt=True)
rob2 = urx.Robot('10.1.1.5', use_rt=True)
conveyer = Convenor(rob1)

v = 0.8
a = 0.5

useCamera = False

pos = Vec2(float(25/1000), float(-385/10000))
lastPos = Vec2(pos.x, pos.y)

objectLocated = 0
objectCount = 0

overBlock = 0.25
atBlock = 0.17

overConveyer = 0.20
atConveyer = 0.148

# clearCamera = 0.25, -0.22, 0.20, 0, 3.14, 0

idlePose = Pose(0.25, -0.22, 0.25)
placeObject = Vec2(0.3, -0.17)
conveyerPos = Vec2(-0.012, 0.4)

#function for moving robot using moveJ
def move(robot: urx.Robot, location: Pose, moveWait: bool) -> None:
    #moves robot
    robot.movex("movej", location.toTuple(), acc=a, vel=v, wait=moveWait, relative=False, threshold=None)
    if moveWait == False:
        time.sleep(0.1)

#Uses camera to locate objects
def locateObjects() -> Vec2:
    global objectLocated, switchCounter

    # check for response
    page = urllib.request.urlopen('http://10.1.1.8/CmdChannel?TRIG')
    time.sleep(2)

    # Get coords
    page = urllib.request.urlopen('http://10.1.1.8/CmdChannel?gRES')
    
    #reads output from camera
    coords = page.read().decode('utf-8')
    
    print(coords)

    #splits output
    objectLocated = int(coords.split()[1])

    switchCounter = 0

    x, y = coords.split()[2].split(',')

    pos = Vec2((float(x) + 25) / 1000, (float(y) - 385) / 1000)
    
    time.sleep(3)

    return pos

#Moves robot to coordinates set by camera
def moveObject(fromPos: Vec2, toPos: Vec2, fromTable: bool, toTable: bool) -> None:
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
    # Placed object


if __name__ == '__main__':
    move(rob1, idlePose, True)

    # print(conveyer.getDistance(4))

    if useCamera:
        pos = locateObjects()
        time.sleep(5)
    else: 
        pos = Vec2(0.00773, -0.31881)

    moveObject(pos, conveyerPos, True, False)

    # print(conveyer.getDistance(4))

    move(rob1, idlePose, True)

    moveObject(conveyerPos, pos, False, True)

    rob1.close()
    rob2.close()