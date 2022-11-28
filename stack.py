from __future__ import annotations

from copy import deepcopy
from util import Vec2, Vec3, Object


class Stack:
    current_height = 0
    padding = Vec2(0.1, 0.1)

    def __init__(self, coords: Vec2 | Vec3, direction: Vec2 | Vec3, height: int, obj: Object):
        self.coords = coords if type(coords) is Vec3 else coords.to_vec3()
        self.direction = direction if type(direction) is Vec3 else direction.to_vec3()
        self.height = height
        self.object = obj

        self.original_coords = deepcopy(self.coords)

    def next(self) -> Vec2:
        """
        Return next position to store the object.
        Starts at the ´self.coords` coordinates, then moves in `self.coords + object['size'] * direction`.
        """
        return_value = deepcopy(self.coords)

        self.current_height = (self.current_height + 1) % self.height

        self.coords.z = (self.object['size'].z * self.current_height)

        if self.current_height == 0:
            self.coords = self.coords + self.padding + self.object['size']

        return return_value * self.direction

    def reset(self):
        self.coords = deepcopy(self.original_coords)


if __name__ == '__main__':
    stack = Stack(Vec2(0.0, 0.0), Vec2(-1.0, 1.0), 2, Object.CUBE)

    print(stack.next())
    print(stack.next())
    print(stack.next())
    stack.reset()
    print(stack.next())
