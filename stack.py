from __future__ import annotations

from copy import deepcopy
from util import Vec2, Vec3, Object


class Stack:
    current_height = 0
    padding = Vec2(0.01, 0.01)
    prev_positions: list[Vec2 | Vec3] = []

    def __init__(self, coords: Vec2 | Vec3, direction: Vec2, height: int, obj: Object):
        self.coords = coords if type(coords) is Vec3 else coords.to_vec3()
        self.direction = direction
        self.height = height
        self.object = obj

        self.original_coords = deepcopy(self.coords)

    def next(self) -> Vec3:
        """
        Return next position to store the object.
        Starts at the ´self.coords` coordinates, then moves in `self.coords + object['size'] * direction`.
        """
        return_value = deepcopy(self.coords)

        self.current_height = (self.current_height + 1) % self.height

        self.coords.z = self.object['size'].z * self.current_height

        if self.current_height == 0:
            self.coords = self.coords + ((self.padding + self.object['size'].to_vec2()) * self.direction)

        self.prev_positions.append(return_value)

        return return_value

    def prev(self) -> Vec3:
        pos = self.prev_positions.pop()

        self.coords = pos

        return pos

    def reset(self):
        self.coords = deepcopy(self.original_coords)


if __name__ == '__main__':
    stack = Stack(Vec2(0.27, -0.42), Vec2(1.0, 0.0), 2, Object.CUBE)

    print(stack.next())
    print(stack.next())
    print(stack.next())

    print()

    print(stack.prev())
    print(stack.prev())
    print(stack.prev())

