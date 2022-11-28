from camera import Camera
from util import Vec2, Object

camera1 = Camera('10.1.1.8', Vec2(240, -170), Vec2(1.08, 1.08), Vec2(-1, -1),
                 (Vec2(14, 191), Vec2(450, 325)), 250,
                 [Object.CUBE, Object.CYLINDER])
"""Camera for robot 1"""

camera2 = Camera('10.1.1.7', Vec2(-638, -240), Vec2(0.91, 0.91), Vec2(1, 1),
                 (Vec2(0, 70), Vec2(450, 477)), 85,
                 [Object.CUBE, Object.CYLINDER])
"""Camera for robot 2"""

if __name__ == '__main__':
    cam = camera2

    img = cam.get_image()
    thresh = cam.image_to_threshold(img)

    cam.show_image(img)
    # cam.show_image(thresh)

    cubes = cam.get_cubes()
    if cubes:
        print(f'Cubes {len(cubes)}:')
        for cube in cubes:
            print(cube)

    cylinders = cam.get_cylinders()
    if cylinders:
        print(f'\nCylinders {len(cylinders)}:')
        for cylinder in cylinders:
            print(cylinder)
