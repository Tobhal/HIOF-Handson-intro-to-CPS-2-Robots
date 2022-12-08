from __future__ import annotations

import operator
import time
from copy import deepcopy
from threading import Thread, Barrier
from typing import Callable, Optional

from Gripper import *
from camera import Camera
from conveyor import Conveyor
from robot import Robot
from stack import Stack
from util import Vec2, Vec3, Pose, Object, RobotPickUp, Status

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
        'place': Vec3(-0.293, -0.13, 0.005)
    },
    Object.CUBE: Object.CUBE,
    Object.CYLINDER: Object.CYLINDER
}

rob2_cords = {
    'conveyor': Pose(0.05, 0.276, -0.02, rx=2.2, ry=2.2),
    'idlePose': Pose(-0.25, -0.12, 0.09),
    'object': {
        'get': Vec3(0.00773, -0.31881, 0.0),
        'place': Vec3(0.27, -0.11, -0.005),
    },
    Object.CUBE: Object.CUBE,
    Object.CYLINDER: Object.CYLINDER
}

end_program = False

rob1: Optional[Robot] = None
rob2: Optional[Robot] = None

rob1_place_stack = Stack(name='r1_PS',
                         coords=rob1_cords['object']['place'],
                         direction=Vec2(0.0, -1.0),
                         height=2,
                         obj=Object.CUBE)
rob1_conveyor_stack = Stack(name='r1_CS',
                            coords=rob1_cords['conveyor'].to_vec3(),
                            direction=Vec2(0.0, 1.0),
                            height=1,
                            obj=Object.CYLINDER)

rob2_place_stack = Stack(name='r2_PS',
                         coords=rob2_cords['object']['place'],
                         direction=Vec2(0.0, -1.0),
                         height=2,
                         obj=Object.CYLINDER)
rob2_conveyor_stack = Stack(name='r2_CS',
                            coords=rob2_cords['conveyor'].to_vec3(),
                            direction=Vec2(0.0, 1.0),
                            height=1,
                            obj=Object.CUBE)

barrier = Barrier(2)

# noinspection PyBroadException
try:
    rob1 = Robot(host='10.1.1.6',
                 name='rob1',
                 object_store=Object.CUBE,
                 cords=rob1_cords,
                 place_stack=rob1_place_stack,
                 conveyor_stack=rob1_conveyor_stack,
                 barrier=barrier,
                 use_rt=True)

    rob2 = Robot(host='10.1.1.5',
                 name='rob2',
                 object_store=Object.CYLINDER,
                 cords=rob2_cords,
                 place_stack=rob2_place_stack,
                 conveyor_stack=rob2_conveyor_stack,
                 barrier=barrier,
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
def pre_run():
    global rob1, rob2, camera1, camera2, object_move, object_Pick_Up, objects_found
    rob1.move(rob1.cords['idlePose'])
    rob2.move(rob2.cords['idlePose'])

    # Wait for both robots to reach idle state before beginning
    while rob1.status == Status.MOVING and rob2.status == Status.MOVING:
        pass

    objects_found['rob1'][rob1.object_move] = camera1.get_object(rob1.object_move)
    objects_found['rob1'][rob1.object_store] = camera1.get_object(rob1.object_store)

    objects_found['rob2'][rob2.object_move] = camera2.get_object(rob2.object_move)
    objects_found['rob2'][rob2.object_store] = camera2.get_object(rob2.object_store)

    print(f"{objects_found['rob1'][rob1.object_move]=}")
    print(f"{objects_found['rob1'][rob1.object_store]=}")
    print(f"{objects_found['rob2'][rob2.object_move]=}")
    print(f"{objects_found['rob2'][rob2.object_store]=}")

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
def move(rob: Robot, camera: Camera, pre_run_func: Callable[[], None] = None):
    global object_Pick_Up, object_move, rob1, rob2

    idle_count = 0

    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    # Wait for both robots to reach idle state before beginning
    # while rob1.status == Status.MOVING or rob2.status == Status.MOVING:
    #     pass

    while termination_condition():
        # Move objects to conveyor
        if object_move.value == rob.name and Conveyor.status == Status.READY:
            rob.log(f'Move object ({rob.object_move.name}) to conveyor')

            added_offset = Vec3(0.0, 0.0, 0.0)
            if rob.object_move == Object.CYLINDER:
                added_offset.x = 0.04

            rob1.conveyor_stack.object = rob1.object_move
            rob2.conveyor_stack.object = rob2.object_move

            Conveyor.status = Status.WAIT

            for i, obj in enumerate(objects_found[rob.name][rob.object_move]):
                if i > 4:
                    break

                rob.move_object_to_conveyor(objects_found[rob.name][rob.object_move][i].to_vec3() + added_offset,
                                            rob.object_move)
                Conveyor.number_of_items_on_belt += 1

                if rob.name == 'rob1':
                    rob2.conveyor_stack.next()
                else:
                    rob1.conveyor_stack.next()

            Conveyor.status = Status.READY
            object_move = RobotPickUp.NONE
            object_Pick_Up = RobotPickUp.flip(rob.name)

            rob.move(rob.cords['idlePose'])

            idle_count = 0

        # Sort own objects while the other robot is moving its objects
        elif object_move.value == RobotPickUp.flip(rob.name).value and Conveyor.status != Status.MOVING \
                and rob.status == Status.READY and len(objects_found[rob.name][rob.object_store]) > 0:
            rob.log(f'Sort own objects')

            added_offset = Vec3(0.0, 0.0, 0.0)
            if rob.object_move == Object.CYLINDER:
                added_offset.z = 0.05

            rob.pick_object(objects_found[rob.name][rob.object_store][0].to_vec3(), rob.object_store)

            pos = rob.place_stack.next()

            rob.place_object(pos.to_pose() + added_offset, rob.object_store, end_over_object=False)
            rob.move(rob.cords['object']['place'].to_pose() +
                     Vec3(0.0, -0.1, 0.1 + (rob.object_store['size'].z / 2)))

            rob.move(rob.cords['idlePose'])

            idle_count = 0

        # Moving object with the conveyor are done. Run `pre_run` again. Wait for both robots to reach this location
        elif object_move == RobotPickUp.NONE and Conveyor.status == Status.READY and rob.status == Status.READY \
                and idle_count > 0:
            rob.log('Wait for other robot to be done sorting')
            i = rob.barrier.wait()
            if i == 1:
                pre_run_func()

            rob.barrier.wait()

        # This robot is going to pick up the object on the conveyor
        elif object_Pick_Up.value == rob.name:
            # Move above conveyor to pick up objects, but not pick up object
            if Conveyor.status == Status.MOVING:
                rob.log('Moving above conveyor')
                rob.move(rob.cords['conveyor'] + Vec3(0.0, 0.1, 0.1))
                idle_count = 0

            # Move object form conveyor / sort the blocks on the conveyor
            if Conveyor.status == Status.NOT_READY:
                rob.log(f'Move object ({rob.object_store}) form conveyor')
                for i in range(0, Conveyor.number_of_items_on_belt):
                    rob.move_object_from_conveyor(rob.object_store)
                    rob.move(rob.cords['idlePose'])

                object_Pick_Up = RobotPickUp.NONE
                rob.status = Status.READY
                Conveyor.status = Status.READY

                idle_count = 0

        else:
            objects_found[rob.name][Object.CUBE] = camera.get_cubes()
            objects_found[rob.name][Object.CYLINDER] = camera.get_cylinders()

            rob.log(f'{object_move=}, {Conveyor.status=}, {rob.status=}, {idle_count=}')

            idle_count += 1
            time.sleep(1)
        # 1. tell antal objecter på vært bord
        # 2. Har bor 1 flest sirkler eller bor 2 flest firkanter
        # # 2.1. Den som har minst objecter å flytte og minst egene å sortere begynner.
        # # 2.2. Den som har minst å sortere begynner å flytte


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
            # object_Pick_Up = RobotPickUp.R2

            # Wait for object to be picked up
            Conveyor.block_for_detect_object(1, operator.lt)

            # object_move = RobotPickUp.R2
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
            # object_Pick_Up = RobotPickUp.R1

            # Wait for object to be picked up
            Conveyor.block_for_detect_object(4, operator.lt)

            # object_move = RobotPickUp.R1
            Conveyor.status = Status.READY


# Other functions
def main(
        move_func: Callable[[Robot, Camera], None] = None,
        robots: list[tuple[Robot, Camera]] = None,
        conveyor_func: Callable[[], None] = None,
        pre_run_func: Callable[[], None] = None):
    if pre_run_func:
        pre_run_func()

    threads: list[Thread] = list()

    if move_func and robots:
        for rob in robots:
            threads.append(Thread(target=move_func, args=(rob[0], rob[1], pre_run_func)))

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
        main(move, [(rob1, camera1), (rob2, camera2)], conveyor_move, pre_run)
        # main(conveyor_func=conveyor_move)
    except KeyboardInterrupt:
        Conveyor.stop()

    print('Program stopped')
    Conveyor.stop()
    rob1.close()
    rob2.close()
