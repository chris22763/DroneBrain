
class Cerebellum ():
    """ das Kleinhirn (Cerebellum) ist für den gleichgewichtssinn und die bewegung sowie deren koordination zuständig """

    def __init__(self):

        self.queue = None
        self.sensor_data = None
        self.target = [1.0, 1.0]
        self.third_dimension = True
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
        lat1 = float(pos1[0])  # lath
        long1 = float(pos1[1])  # long
        lat2 = float(pos2[0])
        long2 = float(pos2[1])

        degree_to_rad = float(math.pi / 180.0)

        d_lat = (lat2 - lat1) * degree_to_rad
        d_long = (long2 - long1) * degree_to_rad

        a = pow(math.sin(d_lat / 2), 2) + math.cos(lat1 * degree_to_rad)
        a *= math.cos(lat2 * degree_to_rad) * pow(math.sin(d_long / 2), 2)

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        m = 6367 * c * 1000  # km
        #mi = 3956 * c

        return m


    @staticmethod
    def calc_direction_in_rad(curr_pos, tar_pos):
        dif_vec = (curr_pos[0] - tar_pos[0], curr_pos[1] - tar_pos[1])
        rad = math.atan2(dif_vec[0], dif_vec[1])

        deg = rad / (math.pi/180)

        return dif_vec, rad, deg


    def over_threshold(self, val, pos, threshold=155):

        size = (424, 240)

        xo = (10*((size[0]/pos[0])**2))
        yo = (10*((size[1]/pos[1])**2))

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
        obst = set()  # Obstacle
        free = set()
        for seed in self.flower:
            val = img[seed[0]][seed[1]]
            if val >= self.over_threshold(val, seed):
                obst.add(seed)
            else:
                free.add(seed)

        return free, obst


    def distance_in_pixel(self, val):
        dim = ((1/(val-256))*-10)-1  # if val 0..255  # -1 to make 1..10m to 0..9m
        # dim = val * 10           # if val 0.0 .. 1.0
        # _d = (int(260/dim), int(120/dim))  # 130x60@2m and 1m x 0.5m 
        _d = (int(130/dim), int(60/dim))  # only half the pixel ammount is needed.
        return _d


    def calculate_vector(self, sensor_data, target):
        import math

        pos_now = sensor_data['GPS']
        pos_tar = target[0]

        distance = self.haversine(pos_now, pos_tar)

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
        self.risk_list = []
        _spiral_len = self.spiral.__len__()
        depth_frame = self.schlafgemach.get_realsense_data()
        depth_np = self.schlafgemach.realsense_to_numpy()

        free, obst = self.check_flower(depth_np)

        potantial_target = set()

        for p in free:
            cell_val = depth_np[p[0]][p[1]]
            d = self.distance_in_pixel(cell_val)
            square = set()
            for x in range(p[0] - d[0], p[0] + d[0]):
                for y in range(p[1] - d[1], p[1] + d[1]):
                    square.add((x, y))

            intersec = square.intersection(obst)

            if len(intersec) == 0:
                potantial_target.add(p)
            elif len(intersec) <= 10:
                for point_intersected in intersec:
                    pass

        if not potantial_target :
            self.rotate_ship(rotation*2)

        else:
            self.fly_through_gate(potantial_target[0])

        # for cord, i in enumerate(self.spiral):
            #
            # chunk = self.schlafgemach.create_chunk(cord[0], cord[1], self.spiral[-1][0], self.spiral[-1][1], depth_np)
            # risk_val = self.calc_risk(cell_val,i ,_spiral_len)
            # risk_val = int((100/_spiral_len) * (_spiral_len - i))
            # if risk_val >= self.min_risk:
                # self.risk_list.append((cell_val, risk_val, i, cord))
                # if risk_val >= self.max_risk:
                    # break

        return correction, rotation


    def fly_to_target(self):
        _sensor_data = self.schlafgemach.sensor_data
        if 'GPS' in _sensor_data:
            while True:  # _sensor_data['GPS'] != self.target[0]:
                correction, rotation = self.calculate_vector(_sensor_data, self.target)
                correction, rotation = self.avoid_obstacle(correction, rotation)
                self.send_course(correction, rotation)

                if self.good_enough(_sensor_data['GPS'], self.target[0], 1/1000):
                    self.target = self.target[1:]
                    break


    def run(self, schlafgemach, queue):
        self.schlafgemach = schlafgemach
        res = self.schlafgemach.resolution
        self.spiral = self.schlafgemach.create_flower(res[0], res[1])
        self.queue = queue

        self.fly_to_target()
