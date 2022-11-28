from __future__ import annotations

import urx
import time

from Gripper import *
from util import Status, Vec2, Vec3, Pose, Object
from threading import Lock


class Robot(urx.Robot):
    a = 0.5
    v = 0.8

    status = Status.NOT_READY

    def __init__(self, host: str, name: str, object_store: Object,
                 cords: dict[str | Object, Pose | dict[str, Vec3] | dict[str, Vec3] | dict[str, Vec3]],
                 use_rt=False, use_simulation=False):
        super().__init__(host, use_rt, use_simulation)
        self.name = name
        self.object_store = object_store
        self.object_move = Object.flip(object_store)
        self.cords = cords
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

    def pick_object(self, location: Pose, current_object=Object.CUBE):
        """
        Pick up an object at a location.
        Default to `CUBE` object
        """
        with self.lock:
            self.send_program(rq_open())
        # time.sleep(0.1)

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
        time.sleep(0.1)

        self.move(location + self.cords[current_object]['over'])

    def center_object(self, location: Pose, current_object=Object.CUBE):
        """
        Center the object in the TCP. Grabs the object at one location, TCP rotates 90 deg and grabs again.
        Uses the location for the centring
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
                self.send_program(rq_close())
            time.sleep(0.5)

            with self.lock:
                self.send_program(rq_open())
            time.sleep(0.5)

            self.move(center_location + self.cords[current_object]['over'])

        open_close()

        center_location.rx = 2.2
        center_location.ry = 2.2

        open_close()

    def move(self, location: Pose, move_wait=True):
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

    def move_object(self, from_pos: Vec3, to_pos: Vec3,
                    current_object=Object.CUBE, stop_at_idle=True, wait_at_idle=False
                    ):
        """
        Move object from position to position. Leaves the robot above the object.
        Default to `CUBE` object
        """
        self.pick_object(from_pos.to_pose(), current_object)

        if stop_at_idle:
            self.move(self.cords['idlePose'], wait_at_idle)

        self.place_object(to_pos.to_pose(), current_object)

    def move_object_to_conveyor(self, pick_pos: Vec3, current_object=Object.CUBE):
        """
        Moves object form `pickPos` to the `conveyor` position.
        Default to `CUBE` object
        """
        self.object_move(pick_pos, self.cords['conveyor'] + Vec3(0.0, 0.0, 0.001), current_object)

    def move_object_from_conveyor(self, current_object=Object.CUBE):
        """
        Move object from conveyor to table
        """
        conv = self.cords['conveyor']
        conv.rx = 0.0
        conv.ry = 3.14

        self.object_move(conv, self.cords['object']['place'], current_object)
        self.cords['object']['place'].y += object['size'].y + 0.01

        conv.rx = 2.2
        conv.ry = 2.2
