import random
import math
import numpy as np
import matplotlib as plot
import cv2

import json


def create_chunk(self, x, y, x_max, y_max, ):
    chunk = []
    lim_x = x + self.scale if x + self.scale >= x_max else x_max
    lim_y = y + self.scale if y + self.scale >= y_max else y_max
    for _y in range(y, lim_y):
        for _x in range(x, lim_x):
            chunk.append((_x, _y))
    return chunk


def create_flower(xs, ys, num_pts=1000):

    from numpy import pi, cos, sin, sqrt, arange

    hx = int(xs/2)
    hy = int(ys/2)
    indices = arange(0, num_pts, dtype=float) + 0.5

    r = sqrt(indices/num_pts)
    theta = pi * (1 + 5**0.5) * indices
    # print((r*cos(theta)*hx))
    flower = [(int(x+hx),int(y+hy)) for x, y in zip((r*cos(theta)*hx), (r*sin(theta)*hy))]
    return flower


square = [(x, y) for x in range(130) for y in range(60)]
print(square)
cam = cv2.VideoCapture(0)
ret, frame = cam.read()
# print(frame.shape)
x, y = frame.shape[1], frame.shape[0]
blossom = create_flower(x, y)
cv2.namedWindow("test")

img_counter = 0
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
while True:
    ret, frame = cam.read()
    for i, p in enumerate(blossom):
        cv2.circle(frame, p, 3, BLUE, -1)
    cv2.imshow("test", frame)
    if not ret:
        break
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "opencv_frame_{}.png".format(img_counter)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1

cam.release()

cv2.destroyAllWindows()