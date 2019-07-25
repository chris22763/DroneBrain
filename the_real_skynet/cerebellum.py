# das Kleinhirn (Cerebellum) ist für den gleichgewichtssinn und die bewegung sowie deren koordination zuständig

class Cerebellum ():
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


    def calculate_vector(self):

        pass

        return self.correction, self.rotation


    def send_course(self, correction, rotation ):

        pass

        return correction, rotation


    def fly_to_target(self):
        _sensor_data = self.schlafgemach.sensor_data
        if 'GPS' in _sensor_data:
            while _sensor_data['GPS'] != self.target:
                correction, rotation = self.calculate_vector()
                correction, rotation = self.avoid_obstacle(correction, rotation)
                self.send_course(correction, rotation)


    def run(self, schlafgemach, queue):
        self.spiral = self.schlafgemach.create_search_spiral(self.schlafgemach.resolution)
        self.schlafgemach = schlafgemach
        self.queue = queue

        self.fly_to_target