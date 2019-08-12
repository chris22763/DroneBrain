
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


    def avoid_obstacle(self, correction, rotation):
        self.risk_list = []
        _spiral_len = self.spiral.__len__()
        depth_frame = self.schlafgemach.get_realsense_data()
        depth_np = self.schlafgemach.realsense_to_numpy()

        for cord, i in enumerate(self.spiral):

            cell_val = depth_np[cord[0]][cord[1]]
            risk_val = int((100/_spiral_len) * (_spiral_len - i))

            if risk_val >= self.min_risk:
                self.risk_list.append((cell_val, risk_val, i, cord))
                if risk_val >= self.max_risk:
                    break

        return correction, rotation


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


    def calculate_vector(self, sensor_data, target):
        import math

        pos_now = sensor_data['GPS']
        pos_tar = target

        distance = self.haversine(pos_now, pos_tar)

        dif_vec, rad, deg = self.calc_direction_in_rad(pos_now, pos_tar)

        correction = 0
        rotation = 0

        return correction, rotation


    def send_course(self, correction, rotation ):
        pass
        return correction, rotation


    def fly_to_target(self):
        _sensor_data = self.schlafgemach.sensor_data
        if 'GPS' in _sensor_data:
            while _sensor_data['GPS'] != self.target[0]:
                correction, rotation = self.calculate_vector()
                correction, rotation = self.avoid_obstacle(correction, rotation)
                self.send_course(correction, rotation)



    def run(self, schlafgemach, queue):
        self.spiral = self.schlafgemach.create_search_spiral(self.schlafgemach.resolution)
        self.schlafgemach = schlafgemach
        self.queue = queue

        self.fly_to_target