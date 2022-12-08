from __future__ import annotations

from copy import deepcopy
from typing import Optional

from util import Vec2, Vec3, Object


class Stack:
    def __init__(self, name: str, coords: Vec3, direction: Vec2, height: int, obj: Object):
        self.name = name
        self.coords = coords
        self.direction = direction
        self.height = height
        self.object = obj

        self.current_height = 0
        self.padding = Vec2(0.01, 0.01)
        self.prev_positions: list[Vec3] = []

        self.original_coords = deepcopy(self.coords)

    def next(self) -> Vec3:
        """
        Return next position to store the object.
        Starts at the ´self.coords` coordinates, then moves in `self.coords + object['size'] * direction`.
        """
        return_value = deepcopy(self.coords)

        self.current_height = (self.current_height + 1) % self.height

        self.coords.z = (self.object['size'].z * self.current_height) + self.original_coords.z

        if self.current_height == 0:
            self.coords = self.coords + ((self.padding + self.object['size'].to_vec2()) * self.direction)

        self.prev_positions.append(return_value)

        print(f'{self.name}: {return_value=} {self.current_height=}')

        return return_value

    def prev(self) -> Optional[Vec3]:
        """
        Return the previous added location. If the list is empty returns None
        """
        if len(self.prev_positions) == 0:
            return None

        pos = self.prev_positions.pop()
        self.coords = pos

        self.current_height = (self.current_height - 1) % self.height

        return pos

    def reset(self):
        self.coords = deepcopy(self.original_coords)

    def peak(self) -> Vec3:
        return self.prev_positions[-1]

    def __repr__(self):
        return f'Stack {{len: {len(self.prev_positions)}, Elements: {[element for element in self.prev_positions]}}}'


if __name__ == '__main__':
    pass
