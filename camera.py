from __future__ import annotations

import urllib.request
from typing import Optional

import cv2
import numpy as np

from robot import *


class BankException(Exception):
    pass


class NoResultException(Exception):
    pass


class Camera:
    """
    Camera object to interface with each robot's camera.
    """
    switch_counter = 0
    witch_object = 0
    object_located = 0

    def __init__(self,
                 ip: str,
                 offsets: Vec2,
                 offset_scale: Vec2,
                 invert: Vec2,
                 camera_cut: tuple[Vec2, Vec2],
                 camera_threshold: int,
                 objects: list[Object]):
        self.ip = ip
        self.offset = offsets
        self.offset_scale = offset_scale
        self.invert = invert
        self.camera_cut = camera_cut
        self.objects = objects
        self.camera_threshold = camera_threshold
        # TODO: Actually ping the camera to see if it responds.

    def locate_object(self) -> Optional[Vec2]:
        """
        Give the location of the current object. object referring to what object locator the camera is using at the
        given time.

        TODO: Verify that the `gRES` is returning valid coordinates. If it doesn't, ether return None.
        """
        # check for response
        page = urllib.request.urlopen(f'http://{self.ip}/CmdChannel?TRIG')
        time.sleep(2)

        x, y = 0.0, 0.0

        # Get coords
        while x == 0.0 and y == 0.0:  # FIX: Temp fix for troubleshooting
            print('ping camera')
            page = urllib.request.urlopen(f'http://{self.ip}/CmdChannel?gRES')

            # reads output from camera
            coords = page.read().decode('utf-8')

            if 'rgRES 0 No result available.' in coords:
                raise NoResultException('No result available')

            # splits output
            # print(coords.split()[2])
            error_code, obj, self.object_located, y, x = coords.split()[2].split(',')
            self.witch_object = self.objects[int(obj)]

        # print(f'witch:   {self.witch_object}')
        # print(f'located: {self.object_located}')
        print(f'x = {x}, y = {y}')

        # NOTE: The X,Y coordinated on the camera might be Vec2(-Y, -X) or something.
        # So they are inverted and flipped.

        pos = Vec2((float(y)) / 1000, (float(x)) / 1000)

        time.sleep(3)

        return pos

    def locate_object_2(self) -> Optional[Vec2]:
        req = urllib.request.urlopen(f'http://{self.ip}/LiveImage.jpg')
        arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
        img = cv2.imdecode(arr, -1)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        i = 0

        self.show_image(img)

        return None

    @staticmethod
    def show_image(image):
        cv2.imshow('image', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def get_image(self):
        req = urllib.request.urlopen(f'http://{self.ip}/LiveImage.jpg')
        arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
        img = cv2.imdecode(arr, -1)

        x, y = self.camera_cut[0].to_tuple()
        h, w = self.camera_cut[1].to_tuple()

        img = img[x:w, y:h]

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        return img

    def image_coords_to_robot_coords(self, x: int | float, y: int | float) -> tuple[float, float]:
        x = ((x + self.offset.x) * self.invert.x) * self.offset_scale.x
        y = ((y + self.offset.y) * self.invert.y) * self.offset_scale.y

        return x, y

    def get_cubes(self) -> Optional[list[Vec2]]:
        img = self.get_image()

        _, threshold = cv2.threshold(img, self.camera_threshold, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cubes = []

        for cnt in contours:
            x1, y1 = cnt[0][0]
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(cnt)

                if w < 25 or h < 25:
                    continue

                x, y = self.image_coords_to_robot_coords(x, y)

                cube = Vec2((y + (h / 2)) / 1000, (x + (w / 2)) / 1000)

                cubes.append(cube)

        return cubes if len(cubes) > 0 else None

    def get_cylinders(self) -> Optional[list[Vec2]]:
        img = self.get_image()
        img = cv2.blur(img, (3, 3))
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=50,
                                   param2=30,
                                   minRadius=1,
                                   maxRadius=40
                                   )

        cylinders = []

        if circles is not None:
            circles = np.uint16(np.around(circles))

            for pt in circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]

                a, b = self.image_coords_to_robot_coords(a, b)

                cylinders.append(Vec2(b / 1000, a / 1000))

        return cylinders if len(cylinders) > 0 else None

    def switch_object(self, bank: int):
        """
        Switch what object the camera detects.

        In theory, it changes what locator it uses. So `INT_1_0` is the first object locator in the camera. That means
        that `INT_1_1` is the second object locator.

        TODO: Change function to take channel as parameter to change to a specific object locator.
        TODO: Make `switch_counter` be of type `Object` insteadof `int`.
        """
        res = urllib.request.urlopen(f'http://10.1.1.8/CmdChannel?sINT_1_{bank}')

        if 'Ref bank index is not used.' in res.read().decode():
            raise BankException('Change to empty bank. There are not that many object locators created, try a lower '
                                'number. bang = {bank}')

        time.sleep(3)

        self.witch_object = self.objects[bank]

        print(f"Camera {self.ip}: object switched to {self.witch_object}")


""" Code graveyard
        # list for storing names of shapes
        for contour in contours:

            # here we are ignoring first counter because
            # findcontour function detects whole image as shape
            if i == 0:
                i = 1
                continue

            # cv2.approxPloyDP() function to approximate the shape
            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)

            # using drawContours() function
            cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

            # finding center point of shape
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])

            # putting shape name at center of each shape
            if len(approx) == 3:
                cv2.putText(img, 'Triangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 4:
                cv2.putText(img, 'Quadrilateral', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 5:
                cv2.putText(img, 'Pentagon', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 6:
                cv2.putText(img, 'Hexagon', (x, y),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            else:
                cv2.putText(img, 'circle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
"""
