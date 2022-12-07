from __future__ import annotations
from typing import Callable, Optional

import operator
import time

from util import Vec2, Vec3, Pose, Object, RobotPickUp, Status, Direction
from robot import Robot
from Gripper import *
from camera import Camera
from conveyor import Conveyor
from stack import Stack
from threading import Thread

# TODO: Needs more tuning
camera1 = Camera(ip='10.1.1.8',
                 offsets=Vec2(140, -180),
                 offset_scale=Vec2(1.2, 1.13),
                 invert=Vec2(1, -1),
                 camera_cut=(Vec2(0, 160), Vec2(540, 450)),
                 camera_threshold=25,
                 objects=[Object.CUBE, Object.CYLINDER])
"""Camera for robot 1"""

camera2 = Camera(ip='10.1.1.7',
                 offsets=Vec2(-520, -240),
                 offset_scale=Vec2(1.14, 1.2),
                 invert=Vec2(1, 1),
                 camera_cut=(Vec2(0, 50), Vec2(400, 450)),
                 camera_threshold=25,
                 objects=[Object.CUBE, Object.CYLINDER])
"""Camera for robot 2"""

rob1_cords = {
    'conveyor': Pose(0.015, 0.285, -0.032, rx=2.2, ry=-2.2),
    'idlePose': Pose(0.25, -0.12, 0.09),
    'object': {
        'get': Vec3(0.00564, -0.32577, 0.0),
        'place': Vec3(-0.293, -0.20, 0.005)
    },
    Object.CUBE: Object.CUBE,
    Object.CYLINDER: Object.CYLINDER
}

rob2_cords = {
    'conveyor': Pose(0.05, 0.276, -0.02, rx=2.2, ry=2.2),
    'idlePose': Pose(-0.25, -0.12, 0.09),
    'object': {
        'get': Vec3(0.00773, -0.31881, 0.0),
        'place': Vec3(0.27, -0.18, 0.0),
    },
    Object.CUBE: Object.CUBE,
    Object.CYLINDER: Object.CYLINDER
}

end_program = False

rob1: Optional[Robot] = None
rob2: Optional[Robot] = None

rob1_place_stack = Stack(coords=rob1_cords['object']['place'],
                         direction=Vec2(0.0, -1.0),
                         height=2,
                         obj=Object.CUBE)
rob1_conveyor_stack = Stack(coords=rob1_cords['conveyor'].to_vec3(),
                            direction=Vec2(0.0, 1.0),
                            height=1,
                            obj=Object.CYLINDER)

rob2_place_stack = Stack(coords=rob2_cords['object']['place'],
                         direction=Vec2(0.0, -1.0),
                         height=2,
                         obj=Object.CYLINDER)
rob2_conveyor_stack = Stack(coords=rob2_cords['conveyor'].to_vec3(),
                            direction=Vec2(0.0, 1.0),
                            height=1,
                            obj=Object.CUBE)

# noinspection PyBroadException
try:
    rob1 = Robot(host='10.1.1.6',
                 name='rob1',
                 object_store=Object.CUBE,
                 cords=rob1_cords,
                 place_stack=rob1_place_stack,
                 conveyor_stack=rob1_conveyor_stack,
                 use_rt=True)

    rob2 = Robot(host='10.1.1.5',
                 name='rob2',
                 object_store=Object.CYLINDER,
                 cords=rob2_cords,
                 place_stack=rob2_place_stack,
                 conveyor_stack=rob2_conveyor_stack,
                 use_rt=True)

    Conveyor.robot = rob2
    Conveyor.lock = rob2.lock
except:
    print('Error occurred with initializing robots. Exiting')
    if rob1:
        rob1.close()

    if rob2:
        rob2.close()

    end_program = True
    exit()
    # os.execv(sys.argv[0], sys.argv)

object_Pick_Up = RobotPickUp.NONE
"""Robot to pick up from conveyor"""

object_move = RobotPickUp.NONE
"""Robot to move object to conveyor"""

counter = 0
use_camera = False

objects_found = {
    'rob1': {
        Object.CUBE: [],
        Object.CYLINDER: []
    },
    'rob2': {
        Object.CUBE: [],
        Object.CYLINDER: []
    }
}


def termination_condition():
    return end_program or counter < 2


# Pre robot move functions
def pre_run1():
    global rob1, rob2, camera1, camera2, object_move, object_Pick_Up, objects_found
    # 1. Count number of object on each table
    objects_found['rob1'][Object.CUBE] = camera1.get_cubes()
    objects_found['rob1'][Object.CYLINDER] = camera1.get_cylinders()

    objects_found['rob2'][Object.CUBE] = camera2.get_cubes()
    objects_found['rob2'][Object.CYLINDER] = camera2.get_cylinders()

    # 2. Calculate the total amount of work both robots needs to do.
    rob1_total_work = objects_found['rob1'][Object.CUBE] + objects_found['rob1'][Object.CYLINDER]
    rob2_total_work = objects_found['rob2'][Object.CUBE] + objects_found['rob2'][Object.CYLINDER]

    if not objects_found['rob1'][rob1.object_move] == objects_found['rob2'][rob2.object_move]:
        # 2.1. The robot with the least to do start to move objects
        if rob1_total_work < rob2_total_work:
            object_move = RobotPickUp.R1
        else:
            object_move = RobotPickUp.R2
    else:
        # 2.2. The robot with the least objects to sort starts
        if objects_found['rob1'][rob1.object_store] < objects_found['rob2'][rob2.object_store]:
            object_move = RobotPickUp.R1
        else:
            object_move = RobotPickUp.R2


def pre_run2():
    global rob1, rob2, camera1, camera2, object_move, object_Pick_Up, objects_found
    rob1.move(rob1.cords['idlePose'])
    rob2.move(rob2.cords['idlePose'])

    # Wait for both robots to reach idle state before beginning
    while rob1.status == Status.MOVING and rob2.status == Status.MOVING:
        pass

    objects_found['rob1'][rob1.object_move] = camera1.get_cubes()
    objects_found['rob1'][rob1.object_store] = camera1.get_cylinders()

    objects_found['rob2'][rob2.object_move] = camera2.get_cubes()
    objects_found['rob2'][rob2.object_store] = camera2.get_cylinders()

    if len(objects_found['rob1'][rob1.object_move]) == 0 and len(objects_found['rob2'][rob2.object_move]) == 0:
        print(f'No objects to move')
        object_move = RobotPickUp.NONE
    elif len(objects_found['rob1'][rob1.object_move]) > len(objects_found['rob2'][rob2.object_move]):
        print(f'Rob 1 move object: {rob1.object_move.name} {len(objects_found["rob1"][rob1.object_move])}')
        object_move = RobotPickUp.R1
    else:
        print(f'Rob 2 move object: {rob2.object_move.name} {len(objects_found["rob2"][rob2.object_move])}')
        object_move = RobotPickUp.R2


# Robot move functions
def move1(rob: Robot, camera: Camera):
    """
    Move objects based on their position form the camera.
    """
    global object_Pick_Up, object_move, rob1, rob2

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    # Wait for both robots to reach idle state before beginning
    while rob1.status == Status.MOVING and rob2.status == Status.MOVING:
        pass

    while termination_condition():
        if object_move.value == rob.name:
            rob.move_object_to_conveyor(objects_found[rob.name][rob.object_move][0].to_vec3(), rob.object_move)

            if rob.name == 'rob1':
                rob2.conveyor_stack.next()
            else:
                rob1.conveyor_stack.next()

            object_move = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])

        if object_Pick_Up.value == rob.name:
            rob.move(rob.cords['conveyor'] + Vec3(0.0, 0.1, 0.1))

            rob.move_object_from_conveyor()

            object_Pick_Up = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])


def move2(rob: Robot, camera: Camera):
    global object_Pick_Up, object_move, rob1, rob2

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    # Wait for both robots to reach idle state before beginning
    # while rob1.status == Status.MOVING or rob2.status == Status.MOVING:
    #     pass

    while termination_condition():
        # Move objects to conveyor
        if object_move.value == rob.name:
            print(f'{rob.name}: Move object')
            number_of_objects = len(objects_found[rob.name][rob.object_move])

            rob1.conveyor_stack.object = rob1.object_move
            rob2.conveyor_stack.object = rob2.object_move

            for i, obj in enumerate(objects_found[rob.name][rob.object_move]):
                if i > 4:
                    break

                rob.move_object_to_conveyor(objects_found[rob.name][rob.object_move][i].to_vec3(), rob.object_move)
                Conveyor.number_of_items_on_belt += 1

                if rob.name == 'rob1':
                    rob2.conveyor_stack.next()
                else:
                    rob1.conveyor_stack.next()

                time.sleep(1)

            Conveyor.status = Status.READY

            object_move = RobotPickUp.NONE
            rob.move(rob.cords['idlePose'])

        # Move object form conveyor / sort the blocks on the conveyor
        elif object_Pick_Up.value == rob.name:
            print(f'{rob.name}: From conveyor')
            for i in range(0, Conveyor.number_of_items_on_belt + 1):
                rob.move(rob.cords['conveyor'] + Vec3(0.0, 0.1, 0.1))

                rob.move_object_from_conveyor()

                rob.move(rob.cords['idlePose'])

            object_Pick_Up = RobotPickUp.NONE
            rob.status = Status.READY

        # Sort own objects while the conveyor is not moving
        elif object_move.value != rob.name and Conveyor.status != Status.MOVING and rob.status == Status.READY:
            print(f'{rob.name}: Sort own block')
            objects_found[rob.name][Object.CUBE] = camera.get_cubes()
            objects_found[rob.name][Object.CYLINDER] = camera.get_cylinders()

            # If there are any objects to store, store them
            if len(objects_found[rob.name][rob.object_store]) > 0:
                rob.pick_object(objects_found[rob.name][rob.object_store][0].to_vec3())

                rob.move(rob.cords['object']['place'].to_vec3() + Vec3(0.0, -0.1, 0.1))
                rob.place_object(rob.place_stack.next().to_pose(), rob.object_store)
                rob.move(rob.cords['object']['place'].to_vec3() + Vec3(0.0, -0.1, 0.1))

                rob.move(rob.cords['idlePose'])
            else:
                time.sleep(2)

        # 1. tell antal objecter på vært bord
        # 2. Har bor 1 flest sirkler eller bor 2 flest firkanter
        # # 2.1. Den som har minst objecter å flytte og minst egene å sortere begynner.
        # # 2.2. Den som har minst å sortere begynner å flytte
        pass


def move_simple(rob: Robot, camera: Camera):
    global object_Pick_Up, object_move, rob1, rob2, objects_found

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    # Wait for both robots to reach idle state before beginning
    while rob1.status == Status.MOVING and rob2.status == Status.MOVING:
        pass

    while termination_condition():
        # Move objects from conveyor
        if object_Pick_Up.value == rob.name:
            with Conveyor.lock:
                for i in range(0, Conveyor.number_of_items_on_belt):
                    conveyor_pos = rob.conveyor_stack.prev().to_vec3() + rob.cords['conveyor'].z

                    if rob.name == 'rob1':
                        rob2.conveyor_stack.prev()
                    else:
                        rob1.conveyor_stack.prev()

                    rob.move_object(conveyor_pos, rob.place_stack.next().to_vec3())

                object_Pick_Up = RobotPickUp.NONE

                rob.move(rob.cords['idlePose'])

        # Move objects to conveyor
        if object_move.value == rob.name and objects_found[rob.name][rob.object_move] > 0:
            obj_found: list[Vec2] = objects_found[rob.name][rob.object_move]

            for i, o in enumerate(obj_found):
                if i > 4:
                    break

                conveyor_pos = rob.conveyor_stack.next().to_vec3()
                conveyor_pos.z = conveyor_pos.z + rob.cords['conveyor'].z

                if rob.name == 'rob1':
                    rob2.conveyor_stack.next()
                else:
                    rob1.conveyor_stack.next()

                rob.move_object(o, conveyor_pos, rob.object_move)
                Conveyor.number_of_items_on_belt += 1

            rob.conveyor_stack.reset()

            rob.move(rob.cords['idlePose'])

        # Sort own objects if other robot is moving objects to conveyor, or the conveyor is not needed
        if object_move.value != rob.name and object_move != RobotPickUp.NONE and Conveyor.status == Status.READY:
            cubes = camera.get_cubes()
            if len(cubes) > 0:
                rob.move_object(cubes[0], rob.place_stack.next(), rob.object_store)
                rob.move(rob.cords['idePose'])


def move_above_objects(rob: Robot, camera: Camera):
    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_close())

    cubes = camera.get_cubes()
    cylinders = camera.get_cylinders()

    print(f'num of cubes = {len(cubes)}')

    if cubes:
        for cube_obj in cubes:
            print(cube_obj)

            rob.move(cube_obj.to_pose() + Vec3(0.0, 0.0, 0.06))

    print(f'num of cylinders = {len(cylinders)}')

    if cylinders:
        for cylinder_obj in cylinders:
            print(cylinder_obj)

            rob.move(cylinder_obj.to_pose() + Vec3(0.0, 0.0, 0.09))

    # rob.pick_object(object_pos.to_pose())
    # rob.place_object(object_pos.to_pose())

    rob.move(rob.cords['idlePose'])


def move_test(rob: Robot, camera: Camera):
    global objects_found

    # for _ in range(0, 4):
    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    # Wait for both robots to reach idle state before beginning
    # while rob1.status == Status.MOVING or rob2.status == Status.MOVING:
    #     pass

    rob.move(rob.cords['conveyor'] + Vec3(0.0, 0.1, 0.1))

    time.sleep(1)

    rob.move(rob.conveyor_stack.next())
    rob.move(rob.conveyor_stack.next())
    rob.move(rob.conveyor_stack.next())

    rob.move(rob.cords['conveyor'] + Vec3(0.0, 0.1, 0.1))

    rob.move(rob.cords['idlePose'])


# Main conveyor move code
def conveyor_move():
    global object_Pick_Up, object_move, end_program

    while termination_condition():
        # object_Pick_Up = RobotPickUp.NONE

        dist_1 = Conveyor.get_distance(1)
        dist_4 = Conveyor.get_distance(4)

        if dist_4 < 50 and Conveyor.status == Status.READY:
            print('conveyor: moving right')
            Conveyor.status = Status.MOVING
            time.sleep(Conveyor.wait_time)

            print(f'set speed = {Conveyor.main_speed}')
            Conveyor.set_speed(Conveyor.main_speed)

            print('start')
            Conveyor.start_right()

            print('wait for detection')
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
        elif dist_1 < 50 and Conveyor.status == Status.READY:
            print('conveyor: moving left')
            Conveyor.status = Status.MOVING

            time.sleep(Conveyor.wait_time)

            print(f'set speed = {Conveyor.main_speed}')
            Conveyor.set_speed(Conveyor.main_speed)
            Conveyor.start_left()

            print('wait for detection')
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
def main(
        move_func: Callable[[Robot, Camera], None] = None,
        robots: list[tuple[Robot, Camera]] = None,
        conveyor_func: Callable[[], None] = None,
        pre_run: Callable[[], None] = None):
    if pre_run:
        pre_run()

    threads: list[Thread] = list()

    if move_func and robots:
        for rob in robots:
            threads.append(Thread(target=move_func, args=(rob[0], rob[1],)))

    if conveyor_func:
        threads.append(Thread(target=conveyor_func))

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()


if __name__ == '__main__':
    print('Program start')

    rob2.set_digital_out(7, 0)  # Make sure the conveyor stop is low before starting.

    try:
        # main(test_move, [(rob1, camera1), (rob2, camera2)], conveyor_move, pre_run1)
        # main(move_above_objects, [(rob1, camera1), (rob2, camera2)])
        main(move2, [(rob1, camera1), (rob2, camera2)], conveyor_move, pre_run2)
        # main(conveyor_func=conveyor_move)
    except KeyboardInterrupt:
        Conveyor.stop()

    print('Program stopped')
    Conveyor.stop()
    rob1.close()
    rob2.close()
