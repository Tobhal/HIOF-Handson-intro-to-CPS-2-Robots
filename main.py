import time

from Gripper import *
from util import *
from threading import Thread
            
camera1 = Camera('10.1.1.8')
camera2 = Camera('10.1.1.7')

block = {
    'over': Vec3(0.0, 0.0, 0.9),
    'at': Vec3(0.0, 0.0, 0.0)
}

cylinder = {    # TODO: Change to correct measurements
    'over': Vec3(0.0, 0.0, 0.9),
    'at': Vec3(0.0, 0.0, 0.0)
}

rob1Cords = {
    'conveyor': Pose(0.015, 0.4, -0.02, rx=2.2, ry=-2.2),
    'idlePose': Pose(0.25, -0.22, 0.09),
    'object': {
        'get': Vec3(0.00773, -0.31881, 0.0),
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
objectMove = RobotPickUp.R1
endProgram = False

counter = 0

useCamera = False

terminationCondition = lambda : endProgram or counter < 2

def rob1Move():
    global rob1, camera1

    print(f'{rob1.name}: move to idle pos = {rob1.cords["idlePose"]}')
    rob1.move(rob1.cords['idlePose'])

    rob1.moveObject(rob1.cords['object']['get'], rob1.cords['conveyor'])
    
    print(f'{rob1.name}: move to idle pos = {rob1.cords["idlePose"]}')
    rob1.move(rob1.cords['idlePose'])

    rob1.cords['object']['get'].z = 0.002 # temp fix

    rob1.moveObject(rob1.cords['conveyor'], rob1.cords['object']['get'])

    rob1.move(rob1.cords['idlePose'])

def rob1Move2():
    global rob1, camera1, objectPickUp, objectMove

    rob1.move(rob1.cords['idlePose'])

    while not terminationCondition:
        if objectPickUp == RobotPickUp.R1:
            rob1.moveObjectFromConveyor()
            objectMove = RobotPickUp.R2
        
        if objectMove == RobotPickUp.R1:
            rob1.moveObjectToConveyor()
            objectMove = RobotPickUp.NONE

def rob2Move():
    global rob2, camera2
    print(f'{rob2.name}: move to idle pos = {rob2.cords["idlePose"]}')
    rob2.move(rob2.cords['idlePose'])

    rob2.moveObject(rob2.cords['object']['get'], rob2.cords['conveyor'])
    
    print(f'{rob2.name}: move to idle pos = {rob2.cords["idlePose"]}')
    rob2.move(rob2.cords['idlePose'])

    rob2.cords['object']['get'].z = 0.002 # temp fix

    rob2.moveObject(rob2.cords['conveyor'], rob2.cords['object']['get'])

    rob2.move(rob2.cords['idlePose'])

def rob2Move2():
    global rob2, camera2, objectPickUp, objectMove

    rob2.move(rob2, rob2Cords['idlePose'])

    while not terminationCondition:
        if objectPickUp == RobotPickUp.R2:
            rob2.moveObjectFromConveyor()
            objectMove = RobotPickUp.R1

        if objectMove == RobotPickUp.R2:
            rob2.moveObjectToConveyor()
            objectMove = RobotPickUp.NONE

def move(rob: Robot, camera: Camera):
    print(f'{rob.name}: move to idle pos = {rob.cords["idlePose"]}')
    rob.move(rob.cords['idlePose'])

    rob.moveObject(rob.cords['object']['get'], rob.cords['conveyor'])

    print(f'{rob.name}: move to idle pos = {rob.cords["idlePose"]}')
    rob.move(rob.cords['idlePose'])

    rob.moveObject(rob.cords['conveyor'], rob.cords['object']['get'])

    rob.move(rob.cords['idlePose'])

# Main conveyor move code
def conveyorMove():
    global objectPickUp, endProgram

    while not terminationCondition:
        objectPickUp = RobotPickUp.NONE

        if Conveyor.getDistance(4) < 50:
            print(f'conveyor moving right')
            time.sleep(Conveyor.waitTime)

            Conveyor.setSpeed(Conveyor.mainSpeed)
            Conveyor.start_right()

            # Wait for object to move to the second sensor before reducing the speed
            Conveyor.blockForDetectObject(2)

            time.sleep(Conveyor.waitAfterDetect)

            Conveyor.setSpeed(Conveyor.stopSpeed)

            # Wait for object to get to first sensor before stopping
            Conveyor.blockForDetectObject(1)

            Conveyor.stop()
            objectPickUp = RobotPickUp.R2

            # Wait for object to be picked up
            Conveyor.blockForDetectObject(1, operator.lt)

        elif Conveyor.getDistance(1) < 50:
            print(f'conveyor moving left')
            time.sleep(Conveyor.waitTime)

            Conveyor.setSpeed(Conveyor.mainSpeed)
            Conveyor.start_left()

            # Wait for object to move to the third sensor before reducing the speed
            Conveyor.blockForDetectObject(3)

            time.sleep(Conveyor.waitAfterDetect)
            
            Conveyor.setSpeed(Conveyor.stopSpeed)

            # Wait for object to get to fourth sensor before stopping
            Conveyor.blockForDetectObject(4)

            Conveyor.stop()
            objectPickUp = RobotPickUp.R1

            # Wait for object to be picked up
            Conveyor.blockForDetectObject(4, operator.lt)

# Other functions
def main():
    rob1Thread = Thread(target=rob1Move)
    rob2Thread = Thread(target=rob2Move)

    rob1Thread.start()
    rob2Thread.start()

    rob1Thread.join()
    rob2Thread.join()

def testMain():
    rob1Thread = Thread(target=rob1Move2)
    rob2Thread = Thread(target=rob2Move2)
    conveyorThread = Thread(target=conveyorMove)

    rob2Thread.start()
    rob1Thread.start()
    conveyorThread.start()

    rob1Thread.join()
    conveyorThread.join()
    rob2Thread.join()

def testMain2():
    rob1Thread = Thread(target=move, args=(rob1, camera1,))
    rob2Thread = Thread(target=move, args=(rob2, camera2,))
    conveyorThread = Thread(target=conveyorMove)

    rob2Thread.start()
    rob1Thread.start()
    conveyorThread.start()

    rob1Thread.join()
    conveyorThread.join()
    rob2Thread.join()

if __name__ == '__main__':
    print('Program start')
    time.sleep(1)

    # main()
    # testMain()
    testMain2()
    
    print('Program stopped')
    rob1.close()
    rob2.close()