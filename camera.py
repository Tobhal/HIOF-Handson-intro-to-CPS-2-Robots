from __future__ import annotations

import urllib.request
import cv2
import time
import numpy as np

from typing import NewType
from util import Vec2, Object

Image = NewType('Image', any)


class NumberToLarge(Exception):
    pass


class BankException(Exception):
    pass


class NoResultException(Exception):
    pass


class Camera:
    """
    Camera object to interface with each robot's camera.
    """
    out_of_range_dist = -0.48

    def __init__(
            self,
            ip: str,
            offsets: Vec2,
            offset_scale: Vec2,
            invert: Vec2,
            camera_cut: tuple[Vec2, Vec2],
            camera_threshold: int,
            objects=(Object.CUBE, Object.CYLINDER)
    ):
        self.ip = ip
        self.offset = offsets
        self.offset_scale = offset_scale
        self.invert = invert
        self.camera_cut = camera_cut
        self.objects = objects

        self.switch_counter = 0
        self.witch_object = 0
        self.object_located = 0

        if camera_threshold not in range(0, 257):
            raise NumberToLarge(f'camera_threshold needs to be in range 0..256, current value is {camera_threshold}')

        self.camera_threshold = camera_threshold

        # TODO: Actually ping the camera to see if it responds.

    def get_image(self) -> Image:
        """
        Gets a pre-cut image from the camera.
        """
        req = urllib.request.urlopen(f'http://{self.ip}/LiveImage.jpg')
        arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
        img = cv2.imdecode(arr, -1)

        x, y = self.camera_cut[0].to_tuple()
        h, w = self.camera_cut[1].to_tuple()

        img = img[x:w, y:h]

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        return img

    def image_to_threshold(self, image: Image) -> Image:
        """
        Convert the camera image to a binary image.
        Uses the `camera_threshold` to create a threshold between white and black
        """
        return cv2.threshold(image, self.camera_threshold, 255, cv2.THRESH_BINARY)[1]

    def image_coords_to_robot_coords(self, x: int | float, y: int | float) -> Vec2:
        """
        Convert the coordinates returned form the camera to what the robots can use.
        """
        coords = Vec2(x, y)
        coords = ((coords + self.offset) * self.invert) * self.offset_scale

        return Vec2(coords.y, coords.x)

    def get_cubes(self) -> list[Vec2] | None:
        """
        Extract the position of each cube that the camera can see.
        """
        img = self.get_image()

        threshold = self.image_to_threshold(img)

        contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cubes = []

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(cnt)

                if w < 25 or h < 25:
                    continue

                w_h = Vec2(w, h)

                cube = self.image_coords_to_robot_coords(x, y)

                cube = ((cube + (w_h / 2)) * self.invert) / 1000

                # Not add cubes that are out of range of the robot for the robot
                if cube.y < self.out_of_range_dist:
                    print(f'Cube out of reach: {cube=}')
                    continue

                cubes.append(cube)

        return cubes if len(cubes) > 0 else None

    def get_cylinders(self) -> list[Vec2] | None:
        """
        Extract the position of each cylinder that the camera can see.
        """
        img = self.get_image()
        img = cv2.blur(img, (3, 3))
        circles = cv2.HoughCircles(
            img, cv2.HOUGH_GRADIENT, 1, 20,
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

                cylinder = self.image_coords_to_robot_coords(a, b)

                cylinder = (cylinder / 1000) * self.invert

                # Not add cylinders that are out of range
                if cylinder.y < self.out_of_range_dist:
                    print(f'Cylinder out of reach: {b}, {a}')
                    continue

                cylinders.append(cylinder)

        return cylinders if len(cylinders) > 0 else None

    def get_shapes(self) -> dict[Object, list[Vec2] | None]:
        """
        Get cubes and cylinders that the camera can detect. In a dict.
        """
        return {
            Object.CUBE: self.get_cubes(),
            Object.CYLINDER: self.get_cylinders()
        }

    def get_object(self, _object: Object) -> list[Vec2] | None:
        return self.get_cubes() if _object == Object.CUBE else self.get_cylinders()


if __name__ == '__main__':
    cam = Camera(
        ip='10.1.1.8',
        offsets=Vec2(140, -180),
        offset_scale=Vec2(1.2, 1.13),
        invert=Vec2(1, -1),
        camera_cut=(Vec2(0, 160), Vec2(540, 450)),
        camera_threshold=25
    )

    print(f'{cam.get_cylinders()[0]}')
