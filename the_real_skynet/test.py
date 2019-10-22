import random
import math
# import numpy as np

import json


def create_chunk(self, x, y, x_max, y_max, ):
    chunk = []
    lim_x = x + self.scale if x + self.scale >= x_max else x_max
    lim_y = y + self.scale if y + self.scale >= y_max else y_max
    for _y in range(y, lim_y):
        for _x in range(x, lim_x):
            chunk.append((_x, _y))
    return chunk

def create_search_spiral(self, X, Y):
    _X = int(X / self.scale)
    _Y = int(Y / self.scale)
    _search_array = []
    x = y = 0
    dx = 0
    dy = -1
    cx = int(_X/2)  # center output to middle of image
    cy = int(_Y/2)
    for i in range(max(_X, _Y)**2):
        if (-_X/2 < x <= _X/2) and (-_Y/2 < y <= _Y/2):

            print (x, y)
            _search_array.append(((x + cx) * self.scale, (y + cy) * self.scale))

        if x == y or (x < 0 and x == -y) or (x > 0 and x == 1-y):
            dx, dy = -dy, dx
        x, y = x + dx, y + dy

    self._search_array = _search_array
    return self._search_array.append


# test = create_search_spiral(5, 5)
min_risk = 150
max_risk = 200

# print(test)

# for i, cord in enumerate(test):
#     val = random.randrange(0,255)
#     risk_val = int((100/test.__len__()) * (test.__len__() - i)) + 1
#     clac_risk =  (risk_val * (val - min_risk))
#     # print('{} : {:03d} : {}'.format(val, risk_val, clac_risk ))

# print(test)
# print(test.__len__())
pattern_one = (484, 240)
pattern_two = (480, 320)
# image_one = np.zeros((np.prod(pattern_one), 1), np.float32)
# image_two = np.zeros((np.prod(pattern_two), 1), np.float32)

# send by drone
# data = {
#     'target': 'mothership',     # address, the network knows how to get their
#     'origin': 'drone_024',      # address, the network knows how to get their
#
#     'size': (3, (2,(int, str)),         # to recognize network problems
#              (3, (tuple, float, 3)),     # holds, type or tuple of size
#              (2, ((484, 240), (480, 320)))),
#
#     'data': [[100, 'move_to'],                  # Mission status  e.g. distance, order
#              [(0.0, 0.0, 0.0), 90.0, (0, 0)]    # Sensor data     e.g. gyro, compass, gps
#              [image_one], [image_two]]          # Camera data,    e.g. depth_cam, sec_cam
# }
#
# output_str = json.dumps(data)
#
# # send by mothership
# data = {
#     'target': 'drone_024',
#     'origin': 'mothership',
#
#     'size':[3, (1, tuple), (1, str), (1, list)],
#
#     'data': [(0.0, 0.0),        # target
#              'move_to_over',    # order
#              [0.0, 0.0, 0.0]]   # e.g. waypoints to hit
# }


def ggt(n, m, _print=True):
    # m should allways be bigger than n
    if n > m:
        i = n
        n = m
        m = i

    no_solution = True
    while no_solution:
        # m = x * n + r
        x = int(m / n)
        r = m % n

        if _print:
            calc = '{} = {}*{}+{}'.format(m, x, n, r)
            print(calc)

        if (r == 0):
            no_solution = False
        else:

            m = n
            n = r

    return n

def is_prime(n):
    root = math.floor(math.sqrt(n))
    if n == 2:
        return True
    if n > 2 and n % 2 == 0:
        return False

    for d in range(3, root + 1, 2):
        if n % d == 0:
            return False
    return True

def get_divider(n):
    divider = []

    for i in range(2, n):
        if n % i == 0:
            divider.append(i)
    return divider

def get_dif_divider(_divider):
    for i in _divider:
        for j in reversed(_divider):
            if i != j:
                if ggt(i, j, False) == 1:
                    return [i, j]


def phi(n):
    if n == 1:
        return 1
    if is_prime(n):
        return n-1

    divider = get_dif_divider(get_divider(n))
    if divider:
        c_last = None
        for d in divider:
            c = 0
            for _i in range(1, d):
                if ggt(_i, d, False) == 1:
                    c += 1
            if c_last:
                c_last = c_last * c
            else:
                c_last = c

        return c_last
    else:
        c = 0
        for _i in range(1, n):
            if ggt(_i, n, False) == 1:

                c += 1

        return c

def create_flower(xs, ys, num_pts=100):

    from numpy import pi, cos, sin, sqrt, arange

    # xs = self.img_size[0]
    # ys = self.img_size[1]
    hx = int(xs/2)
    hy = int(ys/2)
    indices = arange(0, num_pts, dtype=float) + 0.5

    r = sqrt(indices/num_pts)
    theta = pi * (1 + 5**0.5) * indices
    flower = []
    for x, y in zip((r*cos(theta)*hx), (r*sin(theta)*hx)):
        flower.append((int(x+hx),int(y+hy)))

    print(flower[-1])
    print(flower[0])
    print(len(flower))

    flower = []

    flower = [(int(x+hx),int(y+hy)) for x, y in zip((r*cos(theta)*hx), (r*sin(theta)*hx))]
    print('-------')
    print(flower[-1])
    print(flower[0])
    print(len(flower))
    return flower

create_flower(848, 480)
