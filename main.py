import time

from Gripper import *
from util import *
from threading import Thread
            
camera1 = Camera('10.1.1.8')
camera2 = Camera('10.1.1.7')

block = {
    'over': Vec3(0.0, 0.0, 0.09),
    'at': Vec3(0.0, 0.0, 0.0)
}

cylinder = {    # TODO: Change to correct measurements
    'over': Vec3(0.0, 0.0, 0.09),
    'at': Vec3(0.0, 0.0, 0.0)
}

rob1Cords = {
    'conveyor': Pose(0.015, 0.4, -0.022, rx=2.2, ry=-2.2),
    'idlePose': Pose(0.25, -0.22, 0.09),
    'object': {
        'get': Vec3(0.00564, -0.32577, 0.0),
        'place': Vec3(0.3, -0.17, 0.0)
    },
    Object.CUBE: block,
    Object.CYLINDER: cylinder
}

rob2Cords = {
    'conveyor': Pose(0.05, 0.4, -0.02, rx=2.2, ry=2.2),
    'idlePose': Pose(-0.25, -0.22, 0.09),
    'object': {
        'get': Vec3(0.00773, -0.31881, 0.0),
        'place': Vec3(0.3, -0.17, 0.0),
    },
    Object.CUBE: block,
    Object.CYLINDER: cylinder
}

rob1 = Robot('10.1.1.6', 'rob1', rob1Cords, use_rt=True)
rob2 = Robot('10.1.1.5', 'rob2', rob2Cords, use_rt=True)

objectPickUp = RobotPickUp.NONE
objectMove = RobotPickUp.R2
endProgram = False

counter = 0

useCamera = False

terminationCondition = lambda : endProgram or counter < 2

def move(rob: Robot, camera: Camera):
    print(f'{rob.name}: move to idle pos = {rob.cords["idlePose"]}')
    rob.move(rob.cords['idlePose'])

    rob.moveObject(rob.cords['object']['get'], rob.cords['conveyor'])

    print(f'{rob.name}: move to idle pos = {rob.cords["idlePose"]}')
    rob.move(rob.cords['idlePose'])

    rob.moveObject(rob.cords['conveyor'], rob.cords['object']['get'])

    rob.move(rob.cords['idlePose'])

def move2(rob: Robot, camera: Camera):
    global objectPickUp, objectMove, rob1, rob2, terminationCondition, counter

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    while terminationCondition():
        if objectPickUp.value == rob.name:
            objectPickUp = RobotPickUp.NONE
            rob.moveObjectFromConveyor()
            rob.move(rob.cords['idlePose'])
            counter += 1

        if objectMove.value == rob.name:
            objectMove = RobotPickUp.NONE
            rob.moveObjectToConveyor(rob.cords['object']['get'])
            rob.move(rob.cords['idlePose'])

# Main conveyor move code
def conveyorMove():
    global objectPickUp, objectMove, endProgram

    while terminationCondition():
        objectPickUp = RobotPickUp.NONE

        if Conveyor.getDistance(4) < 50:
            print(f'conveyor moving right')
            time.sleep(Conveyor.waitTime)

            Conveyor.setSpeed(rob2, Conveyor.mainSpeed)
            Conveyor.start_right(rob2)

            # Wait for object to move to the second sensor before reducing the speed
            Conveyor.blockForDetectObject(2)

            time.sleep(Conveyor.waitAfterDetectRight)

            Conveyor.setSpeed(rob2, Conveyor.stopSpeed)

            # Wait for object to get to first sensor before stopping
            Conveyor.blockForDetectObject(1)

            Conveyor.stop(rob2)
            objectPickUp = RobotPickUp.R2

            # Wait for object to be picked up
            Conveyor.blockForDetectObject(1, operator.lt)

            objectMove = RobotPickUp.R2

        elif Conveyor.getDistance(1) < 50:
            print(f'conveyor moving left')
            time.sleep(Conveyor.waitTime)

            Conveyor.setSpeed(rob2, Conveyor.mainSpeed)
            Conveyor.start_left(rob2)

            # Wait for object to move to the third sensor before reducing the speed
            Conveyor.blockForDetectObject(3)

            time.sleep(Conveyor.waitAfterDetectLeft)
            
            Conveyor.setSpeed(rob2, Conveyor.stopSpeed)

            # Wait for object to get to fourth sensor before stopping
            Conveyor.blockForDetectObject(4)

            Conveyor.stop(rob2)
            objectPickUp = RobotPickUp.R1

            # Wait for object to be picked up
            Conveyor.blockForDetectObject(4, operator.lt)

            objectMove = RobotPickUp.R1

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

if __name__ == '__main__':
    print('Program start')
    # time.sleep(1)

    # main()
    main2()
    
    print('Program stopped')
    rob1.close()
    rob2.close()