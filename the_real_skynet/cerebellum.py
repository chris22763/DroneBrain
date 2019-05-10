# das Kleinhirn (Cerebellum) ist für den gleichgewichtssinn und die bewegung sowie deren koordination zuständig

class Cerebellum ():
    def __init__(self):

        self.queue = None
        self.sensor_data = None
        self.target = [1.0, 1.0]
        self.third_dimension = True

    def avoid_obstacle(self, correction, rotation):
        pass


    def calculate_vector(self):
        pass


    def send_course(self, correction, rotation ):
        pass


    def fly_to_target(self):
        _sensor_data = self.schlafgemach.sensor_data
        if 'GPS' in _sensor_data:
            while _sensor_data['GPS'] != self.target:
                correction, rotation = self.calculate_vector()
                correction, rotation = self.avoid_obstacle(self, correction, rotation)
                self.send_course(self, correction, rotation)


    def run(self, schlafgemach, queue):

        self.schlafgemach = schlafgemach
        self.queue = queue

        self.fly_to_target