from __future__ import annotations

import urx
import time

from Gripper import *
from util import Status, Vec2, Vec3, Pose, Object
from stack import Stack
from threading import Lock


class Robot(urx.Robot):
    a, v = 0.5, 0.8

    status = Status.NOT_READY

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

    def pick_object(self, location: Pose, current_object=Object.CUBE, center_before_move=False):
        """
        Pick up an object at a location.
        Default to `CUBE` object
        """
        with self.lock:
            self.send_program(rq_open())
        # time.sleep(0.1)

        # if center_before_move:
        #    self.center_object(location, current_object)

        print(f'{self.name}: move above pick up = {location}')
        self.move(location + self.cords[current_object]['over'])
        self.move(location + self.cords[current_object]['at'])

        with self.lock:
            self.send_program(rq_close())
        time.sleep(0.6)

        self.move(location + self.cords[current_object]['over'])

    def place_object(self, location: Pose, current_object=Object.CUBE):
        """
        Place a object at a location.
        Default to `CUBE` object
        """
        print(f'{self.name}: move above place = {location}')
        self.move(location + self.cords[current_object]['over'])
        self.move(location + self.cords[current_object]['at'])

        with self.lock:
            self.send_program(rq_open())
        time.sleep(0.2)

        self.move(location + self.cords[current_object]['over'])

    def center_object(self, location: Pose, current_object=Object.CUBE):
        """
        Center the object in the TCP. Grabs the object at one location, TCP rotates 90 deg and grabs again.
        Uses the location for the centring.
        Leaves the robot at the location
        Default to `CUBE` object
        """
        # Remove rotation
        center_location = Pose(location.x, location.y, location.z)

        with self.lock:
            self.send_program(rq_open())

        print(f'{self.name}: center object at = {location}')
        center_location.rx = 0.0
        center_location.ry = 3.14

        def open_close():
            self.move(center_location + self.cords[current_object]['over'])
            self.move(center_location + self.cords[current_object]['at'])

            with self.lock:
                self.send_program(rq_move(60))
            time.sleep(0.5)

            with self.lock:
                self.send_program(rq_open())
            time.sleep(0.5)

            self.move(center_location + self.cords[current_object]['over'])

        open_close()

        center_location.rx = 2.2
        center_location.ry = 2.2

        open_close()

    def move(self, location: Vec2 | Vec3 | Pose, move_wait=True):
        """
        Function for moving robot using moveJ.
        Acquires the thread lock for just the movement of the robot
        """
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
                    current_object=Object.CUBE, stop_at_idle=True, wait_at_idle=False,
                    center_before_move=False):
        """
        Move object from position to position. Leaves the robot above the object.
        Default to `CUBE` object
        """
        self.pick_object(from_pos.to_pose(), current_object, center_before_move)

        if stop_at_idle:
            self.move(self.cords['idlePose'], wait_at_idle)
            time.sleep(0.5)

        self.place_object(to_pos.to_pose(), current_object)

    def move_object_to_conveyor(self, pick_pos: Vec3, current_object=Object.CUBE):
        """
        Moves object form `pickPos` to the `conveyor` position.
        Default to `CUBE` object
        """
        self.pick_object(pick_pos.to_vec3(), current_object)

        self.move(self.cords['idlePose'])

        self.move(self.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))
        self.place_object(self.conveyor_stack.next().to_vec3())
        self.move(self.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))

        time.sleep(1)

        self.move(self.cords['idlePose'])

    def move_object_from_conveyor(self, current_object=Object.CUBE):
        """
        Move object from conveyor to table
        """
        conv = self.conveyor_stack.prev().to_pose()
        conv.rx = 0.0
        conv.ry = 3.14

        self.move(self.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))

        self.pick_object(conv, current_object)

        self.move(self.cords['conveyor'].to_vec3() + Vec3(0.0, 0.1, 0.1))
        self.cords['object']['place'].y += current_object['size'].y + 0.01

        conv.rx = 2.2
        conv.ry = 2.2

        print('move to idle')
        self.move(self.cords['idlePose'])
        time.sleep(1)

        self.move(self.cords['idlePose'] + Vec3(0.0, -0.2, 0.0))

        print('move on')
        self.place_object(self.place_stack.next().to_pose())

        self.move(self.cords['object']['place'] + Vec3(0.0, -0.2, 0.1))
