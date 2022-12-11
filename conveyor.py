from __future__ import annotations

import operator
import time
from threading import Lock
from typing import Optional

import requests

from robot import Robot
from util import Status, Direction


class Conveyor:
    """
    Static class for handling the conveyor.
    """
    main_speed = 0.10
    """Main speed of the conveyor"""
    stop_speed = 0.025
    """Speed before detection from sensor 1 and 4"""

    wait_after_detection = 3.3
    """Wait after sensor 2 or 3 has detected the object"""

    dist_to_wall = 50
    robot: Robot = None
    lock: Lock = None
    status = Status.READY
    move_direction = Direction.NONE

    number_of_items_on_belt = 0 # TODO: Refactor away?

    @staticmethod
    def log(message):
        print('Conveyor:', message)

    @staticmethod
    def get_distance(sensor: int) -> Optional[float]:
        """
        Change port[n] to change sensor. 1 is closest to the door, 4 is the furthest away from the door.
        If sensors return `out of range` returns 0, because it makes the code simpler.
        """
        r = requests.post('http://10.1.1.9', json={"code": "request", "cid": 1, "adr": "/getdatamulti", "data": {
            "datatosend": [f"/iolinkmaster/port[{sensor}]/iolinkdevice/pdin"]
        }})
        res = r.json()
        res1 = res['data']
        data = str(res1)

        if data[53] == "2":
            d = data[68] + data[69]
            return int(d, 16)
        else:
            print("out of range")
            return 0

    @staticmethod
    def start_right():
        """
        Moves the conveyor to the right
        """
        Conveyor.move_direction = Direction.RIGHT
        Conveyor.log('Started to move right')
        with Conveyor.lock:
            Conveyor.robot.set_digital_out(5, 1)
            # allow digital out 5 to stay active for 0.1s
            time.sleep(0.1)
            # set digital out back to 0
            Conveyor.robot.set_digital_out(5, 0)

        return Conveyor

    @staticmethod
    def start_left():
        """
        Moves the conveyor to the left
        """
        Conveyor.move_direction = Direction.LEFT
        Conveyor.log('Started to move left')
        with Conveyor.lock:
            Conveyor.robot.set_digital_out(6, 1)
            # allow digital out 6 to stay active for 0.1s
            time.sleep(0.1)
            # set digital out back to 0
            Conveyor.robot.set_digital_out(6, 0)

        return Conveyor

    @staticmethod
    def start(direction: Direction):
        Conveyor.status = Status.MOVING

        if direction == Direction.RIGHT:
            Conveyor.start_right()
        elif direction == Direction.LEFT:
            Conveyor.start_left()

        return Conveyor

    @staticmethod
    def stop():
        """
        Stops the conveyor.
         Sets status to `NOT_READY` to make the code easier.
        """
        with Conveyor.lock:
            Conveyor.robot.set_digital_out(7, 1)
            # allow digital out 7 to stay active for 0.1s
            time.sleep(0.1)
            # set digital out back to 0
            Conveyor.robot.set_digital_out(7, 0)

        Conveyor.move_direction = Direction.NONE
        Conveyor.status = Status.NOT_READY

        return Conveyor

    @staticmethod
    def set_speed(voltage: float):
        """
        Sets the speed of the conveyor. The speed is given in voltage
        """
        # sets analog out to voltage instead of current
        with Conveyor.lock:
            Conveyor.robot.send_program("set_analog_outputdomain(1, 1)")
            # sets analog out 1 to desired voltage. 0.012 is the slowest speed.
            Conveyor.robot.set_analog_out(1, voltage)

        return Conveyor

    @staticmethod
    def block_for_detect_object(sensor: int, compare=operator.gt, debug_print=False):
        """
        Blocks the thread while waiting for something to move past the given sensor.
        operator.gt = >
        operator.lt = <
        """
        while compare(dist := Conveyor.get_distance(sensor), Conveyor.dist_to_wall):
            if debug_print:
                print('dist =', dist)

        Conveyor.log(f'Sensor ({sensor}) detected block')

        return Conveyor

    @staticmethod
    def sleep(sec: float | int):
        """
        Sleep for sec amount of time
        """
        time.sleep(sec)
        return Conveyor
