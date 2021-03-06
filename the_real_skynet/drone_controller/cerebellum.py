import time
from numpy import pi, cos, sin, sqrt, arctan2
import numpy as np
import cv2
from numba import cuda
import nucleusfastigii


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
        """Calculate the distance in meter between two coordinates on earth, based on the haversine algorithm.
        in nucleusfastigii is a cuda version"""

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

        if val < threshold:
            return True
        else:
            return False


    def check_flower(self, img):
        obst = np.array([]) # Obstacle
        free = []
        treshhold = 1500
        # start = time.time()
        for seed in self.flower:
            try:
                val = img[seed[1]][seed[0]]
                # fit = self.over_threshold(val, seed)
                index = seed[0] * img.shape[0] + seed[1]
                
                if (val != 0 and (val <= treshhold)) :
                    np.append(obst, index)
                elif val > treshhold:
                    free.append(index)
                    # print('{}, {}, {}'.format(index, seed, img.shape))

                # print('{}, {}, {}'.format(index, val, fit))
            except Exception as e:
                print('{}, {}, {}'.format(seed, e))
        return free, obst


    def distance_in_pixel(self, val):

        dim = (val/1000)# 1000 = depth unit  ## dim = distance in meter
        dip = (int(130/dim), int(60/dim))  # 130px => 1m auf x; 60 => 0.5m auf y @848x480

        return dip, dim


    def calculate_vector(self, sensor_data, target):
        pos_now = sensor_data['GPS']
        pos_tar = target

        distance = self.haversine(pos_now, pos_tar)
        # distance = 0
        # nucleusfastigii.haversine_cuda(pos_now, pos_tar, distance)

        dif_vec, rad, deg = self.calc_direction_in_rad(pos_now, pos_tar)

        correction = 0
        rotation = deg

        return correction, rotation


    def send_course(self, correction, rotation ):
        pass
        return correction, rotation


    def rotate_ship(self, cor, rot):
        """ Mii Rotor controller rotate for dir degree """
        pass


    def fly_through_gate(self, target):
        """ Mii fly towards target """
        pass


    def get_best_point(self, pot, cor, rot):
        y_max = 480  # depth_np.shape
        xh = 848 // 2
        last_nearest_point = [0, 0] # 0. => x value 1. y value
        p_return = []
        for i, p in enumerate(pot):
            if p == 0:
                break
            else:
                _x = p // y_max
                _y = p - (_x * y_max)
                y = (1/(y_max//2)) * (_y - (y_max//2))
                x = (1/xh) * (_x - xh)
                if last_nearest_point[0] < x < p or last_nearest_point[0] > x > p:
                    if last_nearest_point[1] < y < p or last_nearest_point[1] > y > p:
                        last_nearest_point = [x, y]
                        p_return = [_x, _y]

        return p_return


    def avoid_obstacle(self, correction, rotation):
        """ calculate obstacle positions and return list of free paths"""
        start = time.time()
        self.risk_list = []
        depth_frame = self.schlafgemach.get_realsense_data(self.schlafgemach.addon_init['realsense'])
        depth_np = self.schlafgemach.realsense_to_numpy(depth_frame)

        print('{}, {}'.format(depth_np.shape, depth_np.max()))
        free, obst = self.check_flower(depth_np)

        potantial_target = np.zeros(free.__len__(), dtype=np.uint16)

        print('#### time 215: {}'.format(time.time()-start))
        start = time.time()

        print(free.__len__())
        """
        
        stream = cuda.stream()
        with stream.auto_synchronize():
        """
        d_free = cuda.to_device(free)
        d_obst = cuda.to_device(obst)
        d_pt = cuda.to_device(potantial_target)
        d_depth_np =  cuda.to_device(depth_np)
            
        threadsperblock = 16
        offset = 1 if free.__len__() >= threadsperblock else 0
        blockspergrid =  ( (free.__len__() + threadsperblock) // threadsperblock) - offset
        # print('{} : [{}][{}]'.format(free.__len__(), threadsperblock, blockspergrid))
        if free.__len__() > 0 :
            nucleusfastigii.check_corridor_kernel[blockspergrid, threadsperblock](d_free, d_obst, d_pt, d_depth_np)
            result_pt = d_pt.copy_to_host()

            print('#### time 239: {}'.format(time.time()-start))
            start = time.time()
            # print('### pot len0: {}'.format(np.count_nonzero(potantial_target)))
            # print('### pot len1: {}'.format(np.count_nonzero(result_pt)))

            if result_pt[0] != 0:
                point = self.get_best_point(result_pt, correction, rotation)
                # self.fly_through_gate(point)
                # print('free: {}, obstacles: {}, potantial targets: {}'.format(len(free), len(obst), len(potantial_target)))

                print(point)

                if self.headless:
                    self.view_points(depth_np, free, obst, potantial_target, self.flower)

            else:
                self.rotate_ship(correction, rotation)

            # else:
                # self.fly_through_gate(potantial_target[0])
                # print('free: {}, obstacles: {}, potantial targets: {}'.format(len(free), len(obst), len(potantial_target)))

                # if self.headless:
                    # self.view_points(depth_np, free, obst, potantial_target, self.flower)

        else:
            print('turn around dude!')

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
        self.flower = self.schlafgemach.create_flower(res[1], res[0])
        self.queue = queue

        self.fly_to_target()
