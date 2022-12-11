from __future__ import annotations

import operator
import time
from threading import Thread, Barrier
from typing import Callable, Optional

from Gripper import *
from camera import Camera
from conveyor import Conveyor
from robot import Robot
from stack import Stack
from util import Vec2, Vec3, Pose, Object, RobotPickUp, Status, Direction

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
    'conveyor': Pose(0.015, 0.285, -0.032),
    'idlePose': Pose(0.25, -0.12, 0.09),
    'object': {
        'get': Vec3(0.00564, -0.32577, 0.0),
        'place': Vec3(-0.293, -0.13, 0.0)
    },
    Object.CUBE: Object.CUBE,
    Object.CYLINDER: Object.CYLINDER
}

rob2_cords = {
    'conveyor': Pose(0.05, 0.276, -0.02),
    'idlePose': Pose(-0.25, -0.12, 0.09),
    'object': {
        'get': Vec3(0.00773, -0.31881, 0.0),
        'place': Vec3(0.27, -0.11, 0.002),
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

barrier1 = Barrier(2)
barrier2 = Barrier(2)

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

object_Pick_Up = RobotPickUp.NONE
"""Robot to pick up from conveyor"""

object_move = RobotPickUp.NONE
"""Robot to move object to conveyor"""

run_pre_run = False
"""If true both robots move away so the pre run function can be ran again"""

counter = 0


def termination_condition():
    return end_program or counter < 2


# Pre robot move functions
def pre_run():
    global rob1, rob2, camera1, camera2, object_move

    rob1_obj = camera1.get_shapes()
    rob2_obj = camera2.get_shapes()

    if not rob1_obj[rob1.object_move] and not rob2_obj[rob2.object_move]:
        print(f'No objects to move')
        object_move = RobotPickUp.NONE

    elif rob1_obj[rob1.object_move] and not rob2_obj[rob2.object_move]:
        object_move = RobotPickUp.R1
    elif not rob1_obj[rob1.object_move] and rob2_obj[rob2.object_move]:
        object_move = RobotPickUp.R2

    elif not rob1_obj[rob1.object_store] and not rob2_obj[rob2.object_store]:
        if len(rob1_obj[rob1.object_move]) > len(rob2_obj[rob2.object_move]):
            object_move = RobotPickUp.R2
        else:
            object_move = RobotPickUp.R1
    elif not rob1_obj[rob1.object_store] and rob2_obj[rob2.object_store]:
        object_move = RobotPickUp.R2
    elif rob1_obj[rob1.object_store] and not rob2_obj[rob2.object_store]:
        object_move = RobotPickUp.R1

    elif len(rob1_obj[rob1.object_store]) < len(rob2_obj[rob2.object_store]):
        print(f'Rob 1 move object: {rob1.object_store.name} {len(rob1_obj[rob1.object_store])}')
        object_move = RobotPickUp.R1
    elif len(rob1_obj[rob1.object_store]) > len(rob2_obj[rob2.object_store]):
        print(f'Rob 2 move object: {rob2.object_store.name} {len(rob2_obj[rob2.object_store])}')
        object_move = RobotPickUp.R2
    elif len(rob1_obj[rob1.object_store]) == len(rob2_obj[rob2.object_store]):
        object_move = RobotPickUp.R1

    print(f'Robot to move over its objects are {object_move=}')


# Robot move functions
def move(rob: Robot, camera: Camera, pre_run_func: Callable[[], None]):
    global object_Pick_Up, object_move, rob1, rob2, barrier1, barrier2, run_pre_run

    idle_count = 0

    # Wait for both robots to reach idle state before beginning
    rob.move(rob.cords['idlePose'])
    rob.send_program(rq_open())

    barrier1.wait()

    while termination_condition():
        if run_pre_run:
            rob.log('Moving to pre run')

            rob.move(rob.cords['idlePose'])
            barrier1.wait()

            if rob.name == 'rob2':
                pre_run_func()

            run_pre_run = False

            barrier2.wait()

        # Make robot move object to conveyor
        if object_move.value == rob.name \
                and Conveyor.status == Status.READY:
            rob.log(f'Moving {rob.object_move.name} to other robot')

            added_offset = Vec3(0.0, 0.0, 0.0) if rob.object_move == Object.CUBE else Vec3(0.04, -0.00, 0.0)

            rob1.conveyor_stack.object = rob.object_move
            rob2.conveyor_stack.object = rob.object_move

            Conveyor.status = Status.WAIT

            i = 0
            while obj := camera.get_object(rob.object_move):
                if i >= 4:
                    break

                rob.move_object_to_conveyor(obj[0].to_vec3() + added_offset, rob.object_move)
                Conveyor.number_of_items_on_belt += 1

                rob.move(rob.cords['idlePose'])

                if rob.name == 'rob1':
                    rob2.conveyor_stack.next()
                else:
                    rob1.conveyor_stack.next()

                i += 1

            Conveyor.status = Status.READY
            object_move = RobotPickUp.NONE
            object_Pick_Up = RobotPickUp.flip(rob.name)

            rob.log(f'{object_Pick_Up=}')

            rob.move(rob.cords['idlePose'])
            pass

        # Pick object form conveyor
        elif object_Pick_Up.value == rob.name:
            if Conveyor.status == Status.MOVING:
                # Move to prepare for pickup
                rob.move(rob.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))
            elif Conveyor.status == Status.NOT_READY:
                # Start to pick object from conveyor
                rob.log(f'Sorting {rob.object_store.name} from conveyor')

                for i in range(0, Conveyor.number_of_items_on_belt):
                    rob.log(f'{i}')
                    rob.log(f'{rob.conveyor_stack=}')

                    rob.place_stack.next()
                    rob.place_stack.prev()

                    rob.move_object_from_conveyor(rob.object_store)
                    rob.move(rob.cords['idlePose'])

                rob1.conveyor_stack.reset()
                rob2.conveyor_stack.reset()

                Conveyor.number_of_items_on_belt = 0

                object_Pick_Up = RobotPickUp.NONE
                rob.status = Status.READY  # TODO: Is this needed?
                Conveyor.status = Status.READY

                run_pre_run = True
            else:
                rob.log(f'{Conveyor.status=}')
                time.sleep(10)

        # The other robot is still moving its objects to the conveyor, so sort its own
        elif object_Pick_Up == RobotPickUp.NONE and object_move.value != rob.name \
                and not run_pre_run:
            sort_own_blocks(rob, camera)

        # Found move object, run pre_run
        elif object_Pick_Up == RobotPickUp.NONE and object_move == RobotPickUp.NONE and \
                (camera.get_object(rob.object_move)):
            rob.log(f'Found object ({rob.object_move.value}) that needs to be moved to the other robot.')
            run_pre_run = True

        # There are no robot set to move an object to the other robot
        else:
            if object_move == RobotPickUp.NONE and object_Pick_Up == RobotPickUp.NONE \
                    and rob1.status == Status.READY and rob2.status == Status.READY:
                # None of the robots are scheduled to do anything

                time.sleep(5)
            else:
                sort_own_blocks(rob, camera)


def sort_own_blocks(rob: Robot, camera: Camera):
    if obj_store := camera.get_object(rob.object_store):
        rob.log(f'Sorting {rob.object_store.name} at {obj_store[0]=}')

        added_offset = Vec3(0.0, 0.0, 0.0) if rob.object_move == Object.CUBE else Vec3(0.0, 0.0, 0.02)

        rob.pick_object(obj_store[0].to_pose(), rob.object_store)

        place_pos = rob.place_stack.next()  # + added_offset

        rob.place_object(place_pos.to_pose(), rob.object_store, end_over_object=False)

        place_pos.z += rob.object_store['size'].z

        rob.move(place_pos)

        rob.move(rob.cords['idlePose'])


# Main conveyor move code
def conveyor_move():
    global object_move

    while termination_condition():
        if Conveyor.status != Status.READY:
            continue

        if Conveyor.get_distance(1) < Conveyor.dist_to_wall:
            sensors = (3, 4)
            direction = Direction.LEFT
        elif Conveyor.get_distance(4) < Conveyor.dist_to_wall:
            sensors = (2, 1)
            direction = Direction.RIGHT
        else:
            continue

        Conveyor.set_speed(Conveyor.main_speed) \
            .start(direction)\
            .block_for_detect_object(sensors[0]) \
            .sleep(Conveyor.wait_after_detection) \
            .set_speed(Conveyor.stop_speed) \
            .block_for_detect_object(sensors[1]) \
            .stop() \
            .block_for_detect_object(sensors[1], operator.lt)

        object_move = RobotPickUp.R1 if direction == Direction.LEFT else RobotPickUp.R2
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

    rob1.move(rob1.cords['idlePose'])
    rob2.move(rob2.cords['idlePose'])

    # Wait for both robots to reach idle state before beginning
    while rob1.status == Status.MOVING and rob2.status == Status.MOVING:
        pass

    try:
        main(move, [(rob1, camera1), (rob2, camera2)], conveyor_move, pre_run)
        # main(conveyor_func=conveyor_move)
    except KeyboardInterrupt:
        Conveyor.stop()

    print('Program stopped')
    Conveyor.stop()
    rob1.close()
    rob2.close()
