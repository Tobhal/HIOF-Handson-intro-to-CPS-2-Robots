import urllib.request
from typing import Optional

import numpy as np
import cv2


def get_image(camera: int, file_path: Optional[str] = None):
    if file_path:
        img = cv2.imread(file_path)
    else:
        req = urllib.request.urlopen(f'http://10.1.1.{camera}/LiveImage.jpg')
        # noinspection DuplicatedCode
        arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
        img = cv2.imdecode(arr, -1)

    x, y, h, w = 0, 0, 0, 0

    if camera == 8:
        x, y = 0, 160
        h, w = 540, 450
    elif camera == 7:
        x, y = 0, 50
        h, w = 400, 450
    return img[x:w, y:h]


img = get_image(8)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

_, threshold = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)


# Detect cubes
def detect_cubes(_img, _threshold):
    contours, hierarchy = cv2.findContours(_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        x1, y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)

            if w < 25 or h < 25:
                continue

            print(int(x + (w / 2)), int(y + (h / 2)))

            ratio = float(w) / h
            if 0.9 <= ratio <= 1.1:
                # img = cv2.drawContours(img, [cnt], -1, (0, 255, 255), 3)
                _img = cv2.circle(_img, (int(x + (w / 2)), int(y + (h / 2))), radius=2, color=(0, 0, 255), thickness=-1)
                _threshold = cv2.circle(_threshold, (int(x + (w / 2)), int(y + (h / 2))), radius=2, color=(0, 0, 255),
                                        thickness=-1)
                # cv2.putText(img, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                # cv2.putText(img, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                _img = cv2.circle(_img, (int(x + (w / 2)), int(y + (h / 2))), radius=2, color=(0, 0, 255), thickness=-1)
                # img = cv2.drawContours(img, [cnt], -1, (0, 255, 0), 3)
                _threshold = cv2.circle(_threshold, (int(x + (w / 2)), int(y + (h / 2))), radius=2, color=(0, 255, 0),
                                        thickness=-1)

    return _img, _threshold

# threshold = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)


# Detect cylinders
def detect_cylinders(_img, _threshold):
    _gray = cv2.cvtColor(_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.blur(_gray, (3, 3))
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 20,
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

            _img = cv2.circle(_img, (int(a), int(b)), radius=r, color=(0, 255, 0), thickness=2)
            _threshold = cv2.circle(_threshold, (int(a), int(b)), radius=2, color=(0, 255, 0), thickness=-1)

    return _img, _threshold

# img2 = camera.get_cubes()


img, threshold = detect_cubes(img, threshold)
img, threshold = detect_cylinders(img, threshold)

# displaying the image after drawing contours
cv2.imshow('image', img)
# cv2.imshow('threshold', threshold)

cv2.waitKey(0)
cv2.destroyAllWindows()
