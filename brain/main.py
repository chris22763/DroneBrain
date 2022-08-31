import multiprocessing as mp
import sys
import yaml

class Main:

    def __init__(self):
        self._config_path = '../data/test_config.yml' # None
        self._config = None
        self.sensor = None
        self.fc = None

    def setup(self):
        if self._config_path != None:
            self.load_config()
        else:
            for a, arg in enumerate(sys.argv):
                if arg.find('-c') > -1:
                    self._config_path = sys.argv[a+1]

                if arg.find('-h') > -1:
                    print('.py  -c starten mit und einem pfad zu einer config')
                    exit()

    def load_config(self):

        with open(self._config_path, 'r') as f:
            conf = yaml.safe_load(f)
            print(conf)
            self._config = conf


    def start(self):
        """ Hier werden alle notwenidigen Processe gestartet und der netzwerkklasse werden alle queues übergeben. """

        q_list = []
        p_list = []

        # init_sensoren
        if self._config['device']['got_sensors']:
            if self._config['sensor']['realsense'] != None:
                print(self._config['sensor']['realsense'])

            if self._config['sensor']['bno555'] != None:
                print(self._config['sensor']['bno555'])

            if self._config['sensor']['oak1'] != None:
                print(self._config['sensor']['oak1'])

            if self._config['sensor']['oak-depth'] != None:
                print(self._config['sensor']['oak-depth'])
            pass

            import sensor_init

            self.sensor = sensor_init.sensor_init()
            queue = mp.Queue()
            q_list.append(queue)
            process = mp.Process(target=self.sensor.run, args=q_list)
            p_list.append(process)
            process.start()


        # init_steuerung

        import flight_controller

        self.fc = flight_controller.flight_controller()
        queue = mp.Queue()
        q_list.append(queue)
        process = mp.Process(target=self.fc.run, args=q_list)
        p_list.append(process)
        process.start()

        # init_image_recognition

        # init_networking

        pass


if __name__ == "__main__":
    hirnstamm = Main()
    hirnstamm.setup()
    hirnstamm.start()
