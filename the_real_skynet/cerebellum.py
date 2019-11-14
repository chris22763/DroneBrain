import time
from numpy import pi, cos, sin, sqrt, arctan2, arcsin
import numpy as np
import cv2
from numba import cuda, jit
import numba
import math

@cuda.jit
def check_corridor_kernel(free, obst, potantial_target, depth_np):

    cell_val = 0
    pos = cuda.grid(1)
    _p = free[pos]
    print(math.floor(_p / depth_np.shape[0]))
    _x = 0
    _y = 0
    _x = int(math.floor(_p / depth_np.shape[0]))
    _y = int(_p - (_x * depth_np.shape[1]))

    if _p:
        cell_val = depth_np[_x, _y]

        potantial_target = check_corridor(_p, cell_val, obst, potantial_target)


@cuda.jit(device=True)
def check_corridor(p, cell_val, obst, potantial_target):

    # dnp = np.ascontiguousarray(depth_np)
    # rf = np.ascontiguousarray(free)
    # ro = np.ascontiguousarray(obst)

    dim = (cell_val/1000)# 1000 = depth unit  ## dim = distance in meter
    dip = (np.int(130/dim), np.int(60/dim))  # 130px => 1m auf x; 60 => 0.5m auf y @848x480
    # shape = (dip*2)
    square = []
    square = np.zeros(1, dtype=np.int32)
    for x in range(dip[0]*2):
        for y in range(dip[1]*2):
            i = x * (dip[1]*2) + y
            if i in obst:
                np.append(square, i)

    if len(square) == 0:
        np.append(potantial_target, p)
    elif len(square) <= 10:
        # pass
        print(len(square))
        # print('{}\t{}\t{}'.format(p, len(intersec), len(potantial_target)))

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


class Cerebellum ():
    """ das Kleinhirn (Cerebellum) ist für den gleichgewichtssinn und die bewegung sowie deren koordination zuständig """

    def __init__(self):

        self.queue = None
        self.sensor_data = None
        self.target = [1.0, 1.0]
        self.third_dimension = True
        self.headless = False
        self.min_risk = 200
        self.max_risk = 400
        self.risk_list = []


    def good_enough(self, v, t, err):
        """ return true if v(alue) is equal to t(arget) within a margin of err(or).
        v & t können vom typ list, int und float sein"""

        if (isinstance(v, list) or isinstance(v, tuple)) and (isinstance(t, list) or isinstance(t, tuple)):
            if len(v) == len(t):
                b = []
                for i in range(len(v)):
                    b.append(v[i] - err < t[i] < v[i] + err)

                bl = [bi for bi in b if bi == True]

                return True if bl == b else False

        elif (isinstance(v, int) or isinstance(v, float)) and (isinstance(t, int) and isinstance(t, float)):
            b = v - err < t < v + err
            return b

            # v 10, t 6/9/11/12, err = 2
            # v - err (8) < t < v + err (12) => 9/11
            # v - err (8) <= t <= v + err (12) => 9/11/12

        else:
            return False


    @staticmethod
    def haversine(pos1, pos2):
        """Calculate the distance in meter between two coordinates on earth, based on the haversine algorithm."""

        # print('{}, {}'.format(pos1, pos2))
        lat1 = float(pos1[0])  # lath
        long1 = float(pos1[1])  # long
        lat2 = float(pos2[0])
        long2 = float(pos2[1])

        degree_to_rad = float(pi / 180.0)

        d_lat = (lat2 - lat1) * degree_to_rad
        d_long = (long2 - long1) * degree_to_rad

        a = pow(sin(d_lat / 2), 2) + cos(lat1 * degree_to_rad)
        a *= cos(lat2 * degree_to_rad) * pow(sin(d_long / 2), 2)

        c = 2 * arctan2(sqrt(a), sqrt(1 - a))
        m = 6367 * c * 1000  # km
        #mi = 3956 * c

        return m


    @staticmethod
    def calc_direction_in_rad(curr_pos, tar_pos):
        dif_vec = (curr_pos[0] - tar_pos[0], curr_pos[1] - tar_pos[1])
        rad = arctan2(dif_vec[0], dif_vec[1])

        deg = rad / (pi/180)

        return dif_vec, rad, deg


    def view_points(self, img, fset, oset, pset, blossom):

        RED = (0, 0, 255)
        GREEN = (0, 255, 0)
        BLUE = (255,0,0)

        img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img_dot = np.zeros((img_rgb.shape[0],img_rgb.shape[1],3), np.uint8)

        # print(img_rgb.shape)
        # print(img_dot.shape)

        for i, p in enumerate(blossom):
            color = RED if p in oset else None
            color = BLUE if p in fset else color
            color = GREEN if p in pset else color

            cv2.circle(img_dot, (p[1],p[0]), 3, color, -1)

        # cv2.namedWindow('targets',cv2.WINDOW_AUTOSIZE)
        cv2.imshow('targets', img_rgb)
        cv2.imshow('dots', img_dot)
        # final = cv2.addWeighted(img_rgb, 0.4, img_dot, 0.1, 0)
        # cv2.imshow('final', final)

        # cv2.resizeWindow('targets', 600, 600)
        cv2.waitKey(1)


    def over_threshold(self, val, pos, threshold=1500):

        size = (1280, 720)

        xm = int(size[0]/2)
        ym = int(size[1]/2)

        xp = pos[0] - xm if (pos[0] - xm) != 0 else 0.001
        yp = pos[1] - ym if (pos[1] - ym) != 0 else 0.001

        xo = 32768 * ((xp/xm)**2)
        yo = 32768 * ((yp/ym)**2)

        threshold = threshold - (xo * yo)

        if val > threshold:
            return True
        else:
            return False


    def check_square(self, p, mx, my, sx, sy):
        square = set()
        for px in range(p[0], p[0] + (sx * mx)):
            for py in range(p[1], p[1] + (sy * my)):
                square.add((px,py))


    def check_flower(self, img):
        obst = []  # Obstacle
        free = []
        # start = time.time()

        for seed in self.flower:
            try:
                val = img[seed[0]][seed[1]]
                fit = self.over_threshold(val, seed)
                index = seed[0] * img.shape[1] + seed[1]
                if val <= fit:
                    obst.append(index)
                else:
                    free.append(index)

                # print('{}, {}, {}'.format(index, val, fit))
            except Exception as e:
                print('{}, {}, {}'.format(seed, fit, e))
        # print(time.time()-start)
        # print(free)
        # print(free)
        return free, obst


    def distance_in_pixel(self, val):

        dim = (val/1000)# 1000 = depth unit  ## dim = distance in meter
        dip = (int(130/dim), int(60/dim))  # 130px => 1m auf x; 60 => 0.5m auf y @848x480

        return dip, dim


    def calculate_vector(self, sensor_data, target):
        pos_now = sensor_data['GPS']
        pos_tar = target

        distance = self.haversine(pos_now, pos_tar)
        # distance = haversine_cuda(pos_now[0], pos_now[1], pos_tar[0], pos_tar[1])

        dif_vec, rad, deg = self.calc_direction_in_rad(pos_now, pos_tar)

        correction = 0
        rotation = 0

        return correction, rotation


    def send_course(self, correction, rotation ):
        pass
        return correction, rotation


    def rotate_ship(self, dir):
        """ Mii Rotor controller rotate for dir degree """
        pass



    def fly_through_gate(self, target):
        """ Mii fly towards target """
        pass


    def avoid_obstacle(self, correction, rotation):
        """ calculate obstacle positions and return list of free paths"""
        start = time.time()
        self.risk_list = []
        depth_frame = self.schlafgemach.get_realsense_data(self.schlafgemach.addon_init['realsense'])
        depth_np = self.schlafgemach.realsense_to_numpy(depth_frame)

        print('{}, {}'.format(depth_np.shape, depth_np.max()))
        free, obst = self.check_flower(depth_np)

        potantial_target = np.zeros(0, dtype=np.uint16)

        print('#### time 215: {}'.format(time.time()-start))
        start = time.time()

        d_free = cuda.to_device(free)
        d_obst = cuda.to_device(obst)
        d_pt = cuda.to_device(potantial_target)

        d_shape = depth_np.shape
        np.ndarray.flatten(depth_np)

        d_depth_np = cuda.to_device(depth_np)

        print(free.__len__())
        check_corridor_kernel(free, obst, potantial_target, depth_np)

        """
        square = set()
        for p in free:
            sub_time = time.time()
            cell_val = depth_np[p[0]][p[1]]

            # generiert korridor
            d, d_val = self.distance_in_pixel(cell_val)

            for x in range(p[0] - d[0], p[0] + d[0]):
                for y in range(p[1] - d[1], p[1] + d[1]):
                    square.add((x, y))

            # print('{}: {}: {} => ({}, {}), ({}, {})'.format(cell_val, d_val, d, p[0] - d[0], p[1] - d[1], p[0] + d[0], p[1] + d[1]))

            intersec = square.intersection(obst)
            square.clear()
            # print('{}, \t{}'.format(intersec.__len__(), time.time()-start))

            if len(intersec) == 0:
                potantial_target.add(p)
            elif len(intersec) <= 10:
                for point_intersected in intersec:
                    pass

            print('### sub time : {}'.format(time.time()-sub_time))
        """

        print('#### time 239: {}'.format(time.time()-start))
        start = time.time()

        print(potantial_target.__len__())
        print(self.headless)
        if not potantial_target:
            print('next please')
            # self.rotate_ship(rotation*2)

        else:
            # self.fly_through_gate(potantial_target[0])
            print('free: {}, obstacles: {}, potantial targets: {}'.format(len(free), len(obst), len(potantial_target)))

            if self.headless:
                self.view_points(depth_np, free, obst, potantial_target, self.flower)
        # for cord, i in enumerate(self.spiral):
            #
            # chunk = self.schlafgemach.create_chunk(cord[0], cord[1], self.spiral[-1][0], self.spiral[-1][1], depth_np)
            # risk_val = self.calc_risk(cell_val,i ,_spiral_len)
            # risk_val = int((100/_spiral_len) * (_spiral_len - i))
            # if risk_val >= self.min_risk:
                # self.risk_list.append((cell_val, risk_val, i, cord))
                # if risk_val >= self.max_risk:
                    # break

        print('#### time 264: {}'.format(time.time()-start))
        return correction, rotation


    def fly_to_target(self):
        _sensor_data = self.schlafgemach.sensor_data
        if 'GPS' in _sensor_data:
            while True:  # _sensor_data['GPS'] != self.target[0]:
                start = time.time()
                correction, rotation = self.calculate_vector(_sensor_data, self.target)
                correction, rotation = self.avoid_obstacle(correction, rotation)
                self.send_course(correction, rotation)

                if self.good_enough(_sensor_data['GPS'], self.target[0], 1/1000):
                    self.target = self.target[1:]

                print('#### fly_to_target loop :\t{} ####'.format(time.time()-start))

    def run(self, schlafgemach, queue):

        self.schlafgemach = schlafgemach
        res = self.schlafgemach.resolution
        self.flower = self.schlafgemach.create_flower(res[0], res[1])
        self.queue = queue

        self.fly_to_target()
