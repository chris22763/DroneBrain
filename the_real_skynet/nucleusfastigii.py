
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
    x_max = depth_np.shape[1]
    _x = 0
    _y = 0
    _x = _p // y_max
    _y = _p - (_x * y_max)

    if _p:
        cell_val = depth_np[_y, _x]
        potantial_target = check_corridor((_x, _y), cell_val, obst, potantial_target, y_max, x_max)


@cuda.jit(device=True)
def check_corridor(p, cell_val, obst, potantial_target, y_max, x_max):

    dim = (cell_val/1000)# 1000 = depth unit  ## dim = distance in meter
    dip = (np.int16(130/dim), np.int16(60/dim))  # 130px => 1m auf x; 60 => 0.5m auf y @848x480
    obst_counter = 0
    x_l = 0
    x_h = 0
    y_l = 0
    y_h = 0
    
    x_l, x_h, y_l, y_h = dip[0] - p[0], dip[0] + p[0], dip[1] - p[1], dip[1] + p[1]
    
    for x in range(x_l if x_l > 0 else 0 , x_h if x_h < x_max else x_max-1):
        for y in range(y_l if y_l > 0 else 0, y_h if y_h < y_max else y_max-1):
            i = x * y_max + y
            for o in obst:
                if i == o:
                    obst_counter += 1

    for pt in range(len(potantial_target)):
        if obst_counter < 10:
            if potantial_target[pt] == 0:
               potantial_target[pt] = p[0] * y_max + p[1]
               break
    print(obst_counter)
    return potantial_target


@cuda.jit
def haversine_kernel(start, end, distance):

    distance = haversine_cuda(start[0], start[1], end[0], end[1])


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
