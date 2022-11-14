import time

from Gripper import *
from util import *
from threading import Thread
            
camera1 = Camera('10.1.1.8')
camera2 = Camera('10.1.1.7')

block = {
    'over': Vec3(0.0, 0.0, 0.09),
    'at': Vec3(0.0, 0.0, 0.0),
    'size': Vec3(0.05, 0.05, 0.05)
}

cylinder = {    # TODO: Change to correct measurements
    'over': Vec3(0.0, 0.0, 0.09),
    'at': Vec3(0.0, 0.0, 0.0),
    'width': Vec3(0.06, 0.06, 0.07)
}

rob1Cords = {
    'conveyor': Pose(0.015, 0.401, -0.022, rx=2.2, ry=-2.2),
    'idlePose': Pose(0.25, -0.22, 0.09),
    'object': {
        'get': Vec3(0.00564, -0.32577, 0.0),
        'place': Vec3(-0.293, -0.410, 0.0)
    },
    Object.CUBE: block,
    Object.CYLINDER: cylinder
}

rob2Cords = {
    'conveyor': Pose(0.05, 0.399, -0.02, rx=2.2, ry=2.2),
    'idlePose': Pose(-0.25, -0.22, 0.09),
    'object': {
        'get': Vec3(0.00773, -0.31881, 0.0),
        'place': Vec3(0.27, -0.42, 0.0),
    },
    Object.CUBE: block,
    Object.CYLINDER: cylinder
}

# Locks for preventing race conditions
r1Lock = Lock()
r2Lock = Lock()

endProgram = False

try: 
    rob1 = Robot('10.1.1.6', 'rob1', rob1Cords, r1Lock, use_rt=True)
    rob2 = Robot('10.1.1.5', 'rob2', rob2Cords, r2Lock, use_rt=True)
except:
    print('Error ocurred with initializing robots. Exiting')
    endProgram = True
    exit()

objectPickUp = RobotPickUp.NONE     
"""Robot to pick up from conveyor""" 

objectMove = RobotPickUp.R1         
"""Robot to move object to conveyor"""

counter = 0
terminationCondition = lambda : endProgram or counter < 2

useCamera = False

def move(rob: Robot, camera: Camera):
    """
    Moves the robot between two points
    """
    print(f'{rob.name}: move to idle pos = {rob.cords["idlePose"]}')
    rob.move(rob.cords['idlePose'])

    rob.moveObject(rob.cords['object']['get'], rob.cords['conveyor'])

    print(f'{rob.name}: move to idle pos = {rob.cords["idlePose"]}')
    rob.move(rob.cords['idlePose'])

    rob.moveObject(rob.cords['conveyor'], rob.cords['object']['get'])

    rob.move(rob.cords['idlePose'])

def move2(rob: Robot, camera: Camera):
    """
    Moves one block between the two robots
    """
    global objectPickUp, objectMove, terminationCondition, counter

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    while terminationCondition():
        if objectPickUp.value == rob.name:
            rob.moveObjectFromConveyor()

            objectPickUp = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])
            counter += 1

        if objectMove.value == rob.name:
            rob.moveObjectToConveyor(rob.cords['object']['get'])

            objectMove = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])

def move3(rob: Robot, camera: Camera):
    """
    One robot moves the object to the conveyor. When the block is moving the same robot sorts the blocks in its workspace.
    The other robot pick up the object form the conveyor, then sorts it. After that it moves a wrong object to the other robot, it there are any
    """
    global objectPickUp, objectMove, rob1, rob2, terminationCondition, counter

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    # Wait for both robots to reach idle state before beginning
    while rob1.status == Status.MOVING and rob2.status == Status.MOVING:
        pass

    while terminationCondition():
        if objectPickUp.value == rob.name:
            rob.moveObjectFromConveyor()
            rob.cords['object']['place'].y += block['size'].y + 0.01

            objectPickUp = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])
            counter += 1

        if objectMove.value == rob.name:
            rob.moveObjectToConveyor(rob.cords['object']['get'])
            rob.cords['object']['get'].y -= block['size'].y

            objectMove = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])

            rob.moveObject(rob.cords['object']['get'], rob.cords['object']['place'], stopAtIdle=False)
            rob.cords['object']['place'].y += block['size'].y + 0.01
            rob.cords['object']['get'].y -= block['size'].y

            rob.move(rob.cords['idlePose'])
        
        if objectMove.value != rob.name and objectMove != RobotPickUp.NONE and Conveyor.status == Status.READY:
            print(f'Object Move = {objectMove}')
            objectMove = RobotPickUp.flip(RobotPickUp(rob.name))

            rob.moveObject(rob.cords['object']['get'], rob.cords['object']['place'], stopAtIdle=False)
            rob.cords['object']['place'].y += block['size'].y + 0.01
            rob.cords['object']['get'].y -= block['size'].y

            rob.move(rob.cords['idlePose'])


# Main conveyor move code
def conveyorMove():
    global objectPickUp, objectMove, endProgram, r2Lock

    while terminationCondition():
        objectPickUp = RobotPickUp.NONE

        if Conveyor.getDistance(4) < 50:
            print('conveyor: moving right')
            Conveyor.status = Status.MOVING
            time.sleep(Conveyor.waitTime)

            Conveyor.setSpeed(rob2, r2Lock, Conveyor.mainSpeed)
            Conveyor.start_right(rob2, r2Lock)

            # Wait for object to move to the second sensor before reducing the speed
            Conveyor.blockForDetectObject(2)
            print('sensor 2: block detected')

            time.sleep(Conveyor.waitAfterDetectRight)

            Conveyor.setSpeed(rob2, r2Lock, Conveyor.stopSpeed)

            # Wait for object to get to first sensor before stopping
            Conveyor.blockForDetectObject(1)
            print('sensor 1: block detected')

            time.sleep(0.4)

            print('conveyor: stopped')
            Conveyor.stop(rob2, r2Lock)
            Conveyor.status = Status.NOT_READY
            objectPickUp = RobotPickUp.R2

            # Wait for object to be picked up
            Conveyor.blockForDetectObject(1, operator.lt)

            objectMove = RobotPickUp.R2
            Conveyor.status = Status.READY

        elif Conveyor.getDistance(1) < 50:
            print('conveyor: moving left')
            Conveyor.status = Status.MOVING
            time.sleep(Conveyor.waitTime)

            Conveyor.setSpeed(rob2, r2Lock, Conveyor.mainSpeed)
            Conveyor.start_left(rob2, r2Lock)

            # Wait for object to move to the third sensor before reducing the speed
            Conveyor.blockForDetectObject(3)
            print('sensor 3: block detected')

            time.sleep(Conveyor.waitAfterDetectLeft)
            
            Conveyor.setSpeed(rob2, r2Lock, Conveyor.stopSpeed)

            # Wait for object to get to fourth sensor before stopping
            Conveyor.blockForDetectObject(4)
            print('sensor 4: block detected')

            print('conveyor: stopped')
            Conveyor.stop(rob2, r2Lock)
            Conveyor.status = Status.NOT_READY
            objectPickUp = RobotPickUp.R1

            # Wait for object to be picked up
            Conveyor.blockForDetectObject(4, operator.lt)

            objectMove = RobotPickUp.R1
            Conveyor.status = Status.READY

# Other functions
def main():
    rob1Thread = Thread(target=move, args=(rob1, camera1,))
    rob2Thread = Thread(target=move, args=(rob2, camera2,))

    rob2Thread.start()
    rob1Thread.start()

    rob1Thread.join()
    rob2Thread.join()

def main2():
    rob1Thread = Thread(target=move2, args=(rob1, camera1,))
    rob2Thread = Thread(target=move2, args=(rob2, camera2,))
    conveyorThread = Thread(target=conveyorMove)

    rob2Thread.start()
    rob1Thread.start()
    conveyorThread.start()

    rob1Thread.join()
    conveyorThread.join()
    rob2Thread.join()

def main3():
    rob1Thread = Thread(target=move3, args=(rob1, camera1,))
    rob2Thread = Thread(target=move3, args=(rob2, camera2,))
    conveyorThread = Thread(target=conveyorMove)

    rob2Thread.start()
    rob1Thread.start()
    conveyorThread.start()

    rob1Thread.join()
    conveyorThread.join()
    rob2Thread.join()

if __name__ == '__main__':
    print('Program start')
    # time.sleep(1)

    try:
        # main()
        # main2()
        main3()
    except KeyboardInterrupt:
        Conveyor.stop(rob2, r2Lock)
    

    print('Program stopped')
    rob1.close()
    rob2.close()