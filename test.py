import urllib.request
import numpy as np
import cv2


req = urllib.request.urlopen(f'http://10.1.1.7/LiveImage.jpg')
# noinspection DuplicatedCode
arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
img = cv2.imdecode(arr, -1)

# img = cv2.imread('response camera 7 2.jpeg')

""" # camera 10.1.1.8
x, y = 14, 191
h, w = 450, 435
""" # camera 10.1.1.7
x, y = 0, 100
h, w = 450, 477


img = img[x:w, y:h]

# img = bg_remove(img)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Detect cubes
_, threshold = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
# threshold = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)

contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

for cnt in contours:
    x1, y1 = cnt[0][0]
    approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
    if len(approx) == 4:
        x, y, w, h = cv2.boundingRect(cnt)

        if w < 25 or h < 25:
            continue

        print('wh', w, h)

        print(int(x + (w / 2)), int(y + (h / 2)))

        ratio = float(w) / h
        if 0.9 <= ratio <= 1.1:
            # img = cv2.drawContours(img, [cnt], -1, (0, 255, 255), 3)
            img = cv2.circle(img, (int(x + (w / 2)), int(y + (h / 2))), radius=2, color=(0, 0, 255), thickness=-1)
            threshold = cv2.circle(threshold, (int(x + (w / 2)), int(y + (h / 2))), radius=2, color=(0, 0, 255),
                                   thickness=-1)
            # cv2.putText(img, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        else:
            # cv2.putText(img, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            img = cv2.circle(img, (int(x + (w / 2)), int(y + (h / 2))), radius=2, color=(0, 0, 255), thickness=-1)
            # img = cv2.drawContours(img, [cnt], -1, (0, 255, 0), 3)
            threshold = cv2.circle(threshold, (int(x + (w / 2)), int(y + (h / 2))), radius=2, color=(0, 255, 0),
                                   thickness=-1)

# Detect cylinders
blur = cv2.blur(gray, (3, 3))
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                           param1=50,
                           param2=30,
                           minRadius=1,
                           maxRadius=40
                           )

if circles is not None:
    circles = np.uint16(np.around(circles))

    for pt in circles[0, :]:
        a, b, r = pt[0], pt[1], pt[2]

        print(a, b, r)

        img = cv2.circle(img, (int(a), int(b)), radius=r, color=(0, 255, 0), thickness=2)
        threshold = cv2.circle(threshold, (int(a), int(b)), radius=2, color=(0, 255, 0), thickness=-1)

""" Camera 8
points = [
    (57, -294),
    (56, -394),
    (-40, -394)
]
points = [
    (-205, -255),
    (-187, -464),
    (105, -465)
]

"""
"""

print()
print()
print()

img = cv2.circle(img, (10, 10), radius=2, color=(0, 0, 255), thickness=1)

for point in points:
    x, y = point

    x = int(x + (41 / 2))
    y = int(y + (41 / 2))

    x += 200
    y += 560

    x *= -1
    y *= -1

    x *= -1.09
    y *= -1.09

    print(x, y, '\n')

    img = cv2.circle(img, (int(y), int(x)), radius=2, color=(0, 0, 255), thickness=2)
"""
"""
i = 0
# list for storing names of shapes
for contour in contours:

    # here we are ignoring first counter because
    # findcontour function detects whole image as shape
    if i == 0:
        i = 1
        continue

    # cv2.approxPloyDP() function to approximate the shape
    approx = cv2.approxPolyDP(
        contour, 0.01 * cv2.arcLength(contour, True), True
    )

    # using drawContours() function
    cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

    x = 0
    y = 0

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
        cv2.putText(img, 'Hexagon', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    else:
        cv2.putText(img, 'circle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
"""

# img2 = camera.get_cubes()

# displaying the image after drawing contours
cv2.imshow('image', img)
# cv2.imshow('threshold', threshold)

cv2.waitKey(0)
cv2.destroyAllWindows()
