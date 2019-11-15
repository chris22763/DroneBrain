
import numpy as np
from numba import cuda, jit
import math
from numpy import pi, cos, sin, sqrt, arcsin

""" Der Nucleus fastigii (dt. Giebelkern), auch Nucleus medialis cerebelli,
    ist ein Kerngebiet im Kleinhirn.
    (...) über den die Stamm- und proximale Körpermuskulatur gesteuert und reguliert wird.
"""

@cuda.jit
def check_corridor_kernel(free, obst, potantial_target, depth_np):

    cell_val = 0
    pos = cuda.grid(1)
    _p = free[pos]
    y_max = depth_np.shape[0]
    _x = 0
    _y = 0
    _x = int(math.floor(_p / y_max))
    _y = int(_p - (_x * y_max))

    if _p:
        cell_val = depth_np[_y, _x]
        potantial_target = check_corridor((_x, _y), cell_val, obst, potantial_target, y_max)


@cuda.jit(device=True)
def check_corridor(p, cell_val, obst, potantial_target, y_max):

    dim = (cell_val/1000)# 1000 = depth unit  ## dim = distance in meter
    dip = (np.int(130/dim), np.int(60/dim))  # 130px => 1m auf x; 60 => 0.5m auf y @848x480
    obst_counter = 0

    for x in range(dip[0] - p[0], dip[0] + p[0]):
        for y in range(dip[1] - p[1], dip[1] + p[1]):
            i = x * y_max + y
            for o in obst:
                if i == o:
                    obst_counter += 1

            for pt in range(len(potantial_target)):
                if potantial_target[pt] == 0:
                    if obst_counter < 100:
                        potantial_target[pt] = i
                        break

    return potantial_target


@cuda.jit(device=True)
def haversine_cuda(s_lat,s_lng,e_lat,e_lng):
    # approximate radius of earth in km
    R = 6373.0
    s_lat = s_lat * pi / 180
    s_lng = s_lng * pi / 180
    e_lat = e_lat * pi / 180
    e_lng = e_lng * pi / 180
    d = sin((e_lat - s_lat)/2)**2 + \
        cos(s_lat)*cos(e_lat) * \
        sin((e_lng - s_lng)/2)**2
    return 2 * R * arcsin(sqrt(d))
