from __future__ import annotations

import urx
import time

from Gripper import *
from util import Status, Vec2, Vec3, Pose, Object
from stack import Stack
from threading import Lock


class Robot(urx.Robot):
    a, v = 0.5, 0.8

    def __init__(
            self, host: str, name: str, object_store: Object,
            cords: dict[str | Object, Pose | dict[str, Vec3] | dict[str, Vec3] | dict[str, Vec3]],
            place_stack: Stack, conveyor_stack: Stack,
            use_rt=False, use_simulation=False):
        super().__init__(host, use_rt, use_simulation)
        self.name = name
        self.object_store = object_store
        self.object_move = Object.flip(object_store)
        self.cords = cords
        self.place_stack = place_stack
        self.conveyor_stack = conveyor_stack
        self.lock = Lock()

        self.status = Status.NOT_READY

        # activates gripper. only needed once per power cycle
        self.send_program(rq_activate())
        time.sleep(2.5)
        # sets speed of gripper to max
        self.send_program(rq_set_speed(250))
        time.sleep(0.1)
        # sets force of gripper to a low value
        self.send_program(rq_set_force(10))
        time.sleep(0.1)
        # sets robot tcp, the distance from robot flange to gripper tips.
        self.set_tcp((0, 0, 0.16, 0, 0, 0))

        self.status = Status.READY

    def log(self, message: str):
        print(f'{self.name}:', message)

    def pick_object(self, location: Pose, current_object, end_over_object=True):
        """
        Pick up an object at a location.
        Default to `CUBE` object
        """
        with self.lock:
            self.send_program(rq_open())
        # time.sleep(0.1)

        # if center_before_move:
        #    self.center_object(location, current_object)

        # self.log(f'Move ({current_object.name}) above {location=}')
        self.move(location + self.cords[current_object]['over'])
        self.move(location + self.cords[current_object]['at'])

        with self.lock:
            self.send_program(rq_close())
        time.sleep(0.6)
        if end_over_object:
            self.move(location + self.cords[current_object]['over'])

    def place_object(self, location: Pose, current_object, end_over_object=True):
        """
        Place an object at a location.
        Default to `CUBE` object
        """
        self.log(f'Place {current_object.name} at {location=} of {type(location)=}')
        self.move(location + self.cords[current_object]['over'])
        self.move(location + self.cords[current_object]['at'])

        with self.lock:
            self.send_program(rq_open())
        time.sleep(0.2)

        if end_over_object:
            self.move(location + self.cords[current_object]['over'])

    def move(self, location: Vec2 | Vec3 | Pose, move_wait=True):
        """
        Function for moving robot using moveJ.
        Acquires the thread lock for just the movement of the robot
        """
        # self.log('Moving to {location=}')
        self.status = Status.MOVING

        if type(location) == Vec2:
            location = location.to_pose()
        elif type(location) == Vec3:
            location = location.to_pose()

        with self.lock:
            # moves robot
            self.movex("movej", location.to_tuple(), acc=self.a, vel=self.a, wait=move_wait, relative=False,
                       threshold=None)

        if move_wait:
            time.sleep(0.1)

        self.status = Status.READY

    def move_object(self, from_pos: Vec3 | Vec2, to_pos: Vec3 | Vec2,
                    current_object, stop_at_idle=True, wait_at_idle=False,
                    center_before_move=False):
        """
        Move object from position to position. Leaves the robot above the object.
        Default to `CUBE` object
        """
        # self.log(f'Move {current_object=} {from_pos=} {to_pos=}')
        self.pick_object(from_pos.to_pose(), current_object, center_before_move)

        if stop_at_idle:
            self.move(self.cords['idlePose'], wait_at_idle)
            time.sleep(0.5)

        self.place_object(to_pos.to_pose(), current_object)

    def move_object_to_conveyor(self, pick_pos: Vec3, current_object):
        """
        Moves object form `pickPos` to the `conveyor` position.
        """
        # self.log(f'Move {current_object=} from conveyor to {pick_pos=}')
        added_offset = Vec3(0.0, 0.0, 0.0)
        if current_object == Object.CYLINDER:
            added_offset.z = 0.01

        self.pick_object(pick_pos.to_pose(), current_object)

        self.move(self.cords['idlePose'])
        self.move(self.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))

        self.place_object(self.conveyor_stack.next().to_pose() + added_offset, current_object)

        self.move(self.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))

        self.move(self.cords['idlePose'])

    def move_object_from_conveyor(self, current_object):
        """
        Move object from conveyor to table
        """
        # self.log(f'Move {current_object=} from conveyor to {self.place_stack.peak()=}')
        obj = self.conveyor_stack.prev()

        self.move(self.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))

        x_offset = -0.04 if current_object == Object.CYLINDER else 0.0
        obj = Pose(obj.x + x_offset, obj.y, obj.z)

        self.pick_object(obj, current_object)

        self.move(self.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))
        self.cords['object']['place'].y += current_object['size'].y + 0.01

        self.move(self.cords['idlePose'])
        time.sleep(1)

        self.move(self.cords['idlePose'] + Vec3(0.0, -0.2, 0.0))

        place_pos = self.place_stack.next()
        place_pos = Pose(place_pos.x, place_pos.y, place_pos.z)

        self.place_object(place_pos, current_object, end_over_object=False)

        self.log(f'{self.cords["object"]["place"]=}')

        self.move(self.cords['object']['place'] + Vec3(0.0, -0.2, 0.1 + (self.object_store['size'].z / 2)))
