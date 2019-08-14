import random
import math
# import numpy as np

import json

def create_search_spiral(X, Y):
    _search_array = []
    x = y = 0
    dx = 0
    dy = -1
    centerX = int(X/2)
    centerY = int(Y/2)
    for i in range(max(X, Y)**2):
        if (-X/2 < x <= X/2) and (-Y/2 < y <= Y/2):
            print (x + centerX, y + centerY)
            _search_array.append((x + centerX, y + centerY))
            # DO STUFF...
        if x == y or (x < 0 and x == -y) or (x > 0 and x == 1-y):
            dx, dy = -dy, dx
        x, y = x + dx, y + dy
    return _search_array

test = create_search_spiral(5, 5)
min_risk = 150
max_risk = 200

# print(test)



import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt


def get_plot(point_list):
    fig, ax = plt.subplots()

    Path = mpath.Path
    # path_data = [(Path.MOVETO, pos) for pos in point_list]
    path_data = []
    for pos in point_list:
        path_data.append((Path.MOVETO, pos))
    print(path_data)
    codes, verts = zip(*path_data)
    path = mpath.Path(verts, codes)
    patch = mpatches.PathPatch(path, facecolor='r', alpha=0.5)
    ax.add_patch(patch)

    # plot control points and connecting lines
    x, y = zip(*path.vertices)
    line, = ax.plot(x, y, 'go-')

    ax.grid()
    ax.axis('equal')
    plt.show()


get_plot(test)


for i, cord in enumerate(test):
    val = random.randrange(0,255)
    risk_val = int((100/test.__len__()) * (test.__len__() - i)) + 1
    clac_risk =  (risk_val * (val - min_risk))
    # print('{} : {:03d} : {}'.format(val, risk_val, clac_risk ))

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

