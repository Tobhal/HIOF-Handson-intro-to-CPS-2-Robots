from __future__ import annotations

import operator
import time

from typing import Callable, Optional

from util import Vec2, Vec3, Pose, Object, RobotPickUp, Status
from robot import Robot
from Gripper import *
from camera import Camera
from conveyor import Conveyor
from threading import Thread

camera1 = Camera('10.1.1.8', Vec2(240, -170), Vec2(1.08, 1.08), Vec2(-1, -1),
                 (Vec2(14, 191), Vec2(450, 325)), 175,
                 [Object.CUBE, Object.CYLINDER])
"""Camera for robot 1"""

camera2 = Camera('10.1.1.7', Vec2(-638, -240), Vec2(0.91, 0.91), Vec2(1, 1),
                 (Vec2(0, 0), Vec2(450, 477)), 100,
                 [Object.CUBE, Object.CYLINDER])
"""Camera for robot 2"""

cube = {
    'over': Vec3(0.0, 0.0, 0.1),
    'at': Vec3(0.0, 0.0, 0.01),
    'size': Vec3(0.05, 0.05, 0.05)
}

cylinder = {  # TODO: Change to correct measurements
    'over': Vec3(0.0, 0.0, 0.1),
    'at': Vec3(0.0, 0.0, 0.01),
    'width': Vec3(0.06, 0.06, 0.07)
}

rob_1_cords = {
    'conveyor': Pose(0.015, 0.401, -0.022, rx=2.2, ry=-2.2),
    'idlePose': Pose(0.25, -0.12, 0.09),
    'object': {
        'get': Vec3(0.00564, -0.32577, 0.0),
        'place': Vec3(-0.293, -0.410, 0.0)
    },
    Object.CUBE: cube,
    Object.CYLINDER: cylinder
}

rob_2_cords = {
    'conveyor': Pose(0.05, 0.399, -0.02, rx=2.2, ry=2.2),
    'idlePose': Pose(-0.25, -0.12, 0.09),
    'object': {
        'get': Vec3(0.00773, -0.31881, 0.0),
        'place': Vec3(0.27, -0.42, 0.0),
    },
    Object.CUBE: cube,
    Object.CYLINDER: cylinder
}

end_program = False

rob1: Optional[Robot] = None
rob2: Optional[Robot] = None

# noinspection PyBroadException
try:
    rob1 = Robot('10.1.1.6', 'rob1', rob_1_cords, use_rt=True)
    rob2 = Robot('10.1.1.5', 'rob2', rob_2_cords, use_rt=True)
    Conveyor.robot = rob2
    Conveyor.lock = rob2.lock
except:
    print('Error occurred with initializing robots. Exiting')
    end_program = True
    exit()

object_Pick_Up = RobotPickUp.NONE
"""Robot to pick up from conveyor"""

object_move = RobotPickUp.R1
"""Robot to move object to conveyor"""

counter = 0
use_camera = False


def termination_condition():
    return end_program or counter < 2


def move(rob: Robot, camera: Camera):
    """
    Moves the robot between two points
    """
    print(f'{rob.name}: move to idle pos = {rob.cords["idlePose"]}')
    rob.move(rob.cords['idlePose'])

    rob.move_object(rob.cords['object']['get'], rob.cords['conveyor'])

    print(f'{rob.name}: move to idle pos = {rob.cords["idlePose"]}')
    rob.move(rob.cords['idlePose'])

    rob.move_object(rob.cords['conveyor'], rob.cords['object']['get'])

    rob.move(rob.cords['idlePose'])


def move2(rob: Robot, camera: Camera):
    """
    Moves one block between the two robots
    """
    global object_Pick_Up, object_move, counter

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    while termination_condition():
        if object_Pick_Up.value == rob.name:
            rob.move_object_from_conveyor()

            object_Pick_Up = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])
            counter += 1

        if object_move.value == rob.name:
            rob.move_object_to_conveyor(rob.cords['object']['get'])

            object_move = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])


def move3(rob: Robot, camera: Camera):
    """
    One robot moves the object to the conveyor. When the block is moving the same robot sorts the blocks in its
    workspace. The other robot pick up the object form the conveyor, then sorts it. After that it moves a wrong
    object to the other robot, it there is any
    """
    global object_Pick_Up, object_move, rob1, rob2, counter

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    # Wait for both robots to reach idle state before beginning
    while rob1.status == Status.MOVING and rob2.status == Status.MOVING:
        pass

    while termination_condition():
        if object_Pick_Up.value == rob.name:
            rob.move_object_from_conveyor()
            rob.cords['object']['place'].y += cube['size'].y + 0.01

            object_Pick_Up = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])
            counter += 1

        if object_move.value == rob.name:
            rob.move_object_to_conveyor(rob.cords['object']['get'])
            rob.cords['object']['get'].y -= cube['size'].y

            object_move = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])

            rob.move_object(rob.cords['object']['get'], rob.cords['object']['place'], stop_at_idle=False)
            rob.cords['object']['place'].y += cube['size'].y + 0.01
            rob.cords['object']['get'].y -= cube['size'].y

            rob.move(rob.cords['idlePose'])

        if object_move.value != rob.name and object_move != RobotPickUp.NONE and Conveyor.status == Status.READY:
            print(f'Object Move = {object_move}')
            object_move = RobotPickUp.flip(RobotPickUp(rob.name))

            rob.move_object(rob.cords['object']['get'], rob.cords['object']['place'], stop_at_idle=False)
            rob.cords['object']['place'].y += cube['size'].y + 0.01
            rob.cords['object']['get'].y -= cube['size'].y

            rob.move(rob.cords['idlePose'])


def test_move(rob: Robot, camera: Camera):
    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_close())

    cubes = camera.get_cubes()
    cylinders = camera.get_cylinders()

    print(f'num of cubes = {len(cubes)}')

    if cubes:
        for cube_obj in cubes:
            print(cube_obj)

            rob.move(cube_obj.to_pose() + Vec3(0.0, 0.0, 0.09))

    print(f'num of cylinders = {len(cylinders)}')

    if cylinders:
        for cylinder_obj in cylinders:
            print(cylinder_obj)

            rob.move(cylinder_obj.to_pose() + Vec3(0.0, 0.0, 0.09))

    # rob.pick_object(object_pos.to_pose())
    # rob.place_object(object_pos.to_pose())

    rob.move(rob.cords['idlePose'])


# Main conveyor move code
def conveyor_move():
    global object_Pick_Up, object_move, end_program, rob2

    while termination_condition():
        object_Pick_Up = RobotPickUp.NONE

        if Conveyor.get_distance(4) < 50 and Conveyor.status == Status.READY:
            print('conveyor: moving right')
            Conveyor.status = Status.MOVING
            time.sleep(Conveyor.wait_time)

            Conveyor.set_speed(Conveyor.main_speed)
            Conveyor.start_right()

            # Wait for object to move to the second sensor before reducing the speed
            Conveyor.block_for_detect_object(2)
            print('sensor 2: block detected')

            time.sleep(Conveyor.wait_after_detect_right)

            Conveyor.set_speed(Conveyor.stop_speed)

            # Wait for object to get to first sensor before stopping
            Conveyor.block_for_detect_object(1)
            print('sensor 1: block detected')

            time.sleep(0.4)

            print('conveyor: stopped')
            Conveyor.stop()
            Conveyor.status = Status.NOT_READY
            object_Pick_Up = RobotPickUp.R2

            # Wait for object to be picked up
            Conveyor.block_for_detect_object(1, operator.lt)

            object_move = RobotPickUp.R2
            Conveyor.status = Status.READY

        elif Conveyor.get_distance(1) < 50 == Conveyor.status == Status.READY:
            print('conveyor: moving left')
            Conveyor.status = Status.MOVING
            time.sleep(Conveyor.wait_time)

            Conveyor.set_speed(Conveyor.main_speed)
            Conveyor.start_left()

            # Wait for object to move to the third sensor before reducing the speed
            Conveyor.block_for_detect_object(3)
            print('sensor 3: block detected')

            time.sleep(Conveyor.wait_after_detect_left)

            Conveyor.set_speed(Conveyor.stop_speed)

            # Wait for object to get to fourth sensor before stopping
            Conveyor.block_for_detect_object(4)
            print('sensor 4: block detected')

            print('conveyor: stopped')
            Conveyor.stop()
            Conveyor.status = Status.NOT_READY
            object_Pick_Up = RobotPickUp.R1

            # Wait for object to be picked up
            Conveyor.block_for_detect_object(4, operator.lt)

            object_move = RobotPickUp.R1
            Conveyor.status = Status.READY


# Other functions
def main(move_func: Callable[[Robot, Camera], None],
         robots: list[tuple[Robot, Camera]],
         conveyor_func: Callable = None,
         ):
    threads: list[Thread] = list()

    for rob in robots:
        threads.append(Thread(target=move_func, args=(rob[0], rob[1],)))

    if conveyor_func:
        threads.append(Thread(target=conveyor_func))

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()


def test_main():
    rob1_thread = Thread(target=test_move, args=(rob2, camera2,))

    rob1_thread.start()

    rob1_thread.join()


if __name__ == '__main__':
    print('Program start')

    try:
        # main(move)
        # main(move2, conveyor_move)
        # main(move3, conveyor_move)
        # test_main()
        main(test_move, [(rob1, camera1), (rob2, camera2)])
    except KeyboardInterrupt:
        Conveyor.stop()

    print('Program stopped')
    Conveyor.stop()
    rob1.close()
    rob2.close()
