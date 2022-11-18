import urllib.request

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

    def __init__(self, ip: str, offsets: Vec2, objects: list[Object]):
        self.ip = ip
        self.offset = offsets
        self.objects = objects
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

        x = 0.0
        y = 0.0

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
            _, obj, self.object_located, y, x = coords.split()[2].split(',')
            self.witch_object = self.objects[int(obj)]

        # print(f'witch:   {self.witch_object}')
        # print(f'located: {self.object_located}')
        print(f'x = {x}, y = {y}')

        # NOTE: The X,Y coordinated on the camera might be Vec2(-Y, -X) or something.
        # So they are inverted and flipped.

        pos = Vec2((float(y) + self.offset.x) / 1000, (float(x) + self.offset.y) / 1000)

        time.sleep(3)

        return pos

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
