# Truncus cerebri oder Hirnstamm
# das wird die Main
# Todo make super job managment

import multiprocessing as mp
import sys
import configparser


class TruncusCerebri:
    """
    Truncus Cerebri oder der Hirnstamm ist bei uns die Main.py.
    Hier werden alle process, basierend auf den daten eines configfiles, gestartet und angesteuert.
    """
    
    def __init__(self):
        self._config_path = ''
        self._config = None
        self.schlafgemach = None
        self.kleinhirn = None
        self.balken = None


    def setup(self):
        for a, arg in enumerate(sys.argv):
            if arg.find('-c') > -1:
                self._config_path = sys.argv[a+1]

            if arg.find('-h') > -1:
                print('.py starten mit -c und einem pfad zu einer config')

            if arg.find('-hc') > -1:
                from numba import cuda
                gpu = cuda.get_current_device()
                print("name = %s" % gpu.name)
                print("maxThreadsPerBlock = %s" % str(gpu.MAX_THREADS_PER_BLOCK))
                print("maxBlockDimX = %s" % str(gpu.MAX_BLOCK_DIM_X))
                print("maxBlockDimY = %s" % str(gpu.MAX_BLOCK_DIM_Y))
                print("maxBlockDimZ = %s" % str(gpu.MAX_BLOCK_DIM_Z))
                print("maxGridDimX = %s" % str(gpu.MAX_GRID_DIM_X))
                print("maxGridDimY = %s" % str(gpu.MAX_GRID_DIM_Y))
                print("maxGridDimZ = %s" % str(gpu.MAX_GRID_DIM_Z))
                print("maxSharedMemoryPerBlock = %s" %
                      str(gpu.MAX_SHARED_MEMORY_PER_BLOCK))
                print("asyncEngineCount = %s" % str(gpu.ASYNC_ENGINE_COUNT))
                print("canMapHostMemory = %s" % str(gpu.CAN_MAP_HOST_MEMORY))
                print("multiProcessorCount = %s" % str(gpu.MULTIPROCESSOR_COUNT))
                print("warpSize = %s" % str(gpu.WARP_SIZE))
                print("unifiedAddressing = %s" % str(gpu.UNIFIED_ADDRESSING))
                print("pciBusID = %s" % str(gpu.PCI_BUS_ID))
                print("pciDeviceID = %s" % str(gpu.PCI_DEVICE_ID))
                
                exit()

        self.load_config()


    def load_config(self):
        parser = configparser.ConfigParser()
        parser.read(self._config_path)
        if not self._config:
            self._config = {}
            for key in parser:
                self._config[key] = parser[key]
                for sub_key in parser[key]:
                    self._config[key][sub_key] = parser[key][sub_key]

        # print(self._config)


    def queue_handler(queue, key=None, data=None):
        """ Erhält kommandos und daten der Prozesse und schickt diese als Dictonary an die in data definierten Adresaten."""
        output = {}

        if data:
            if key :
                output[key] = {
                    'dev_name': data[0],  # 'drone_nr_00',
                    'command': data[1],     # '192.168.2.1,192.168.2.2,192.168.2.3',
                    'data': data[2]
                }

            for q in queue:
                q.put(output)    


    def check_thalamus(self):
        """ init funktion für thalamus.py, basierend auf den configdaten, werden hier alle angeschlossenen Sensoren initialisiert. """

        import thalamus                 # Alle Sensoren

        self.schlafgemach = thalamus.Thalamus()

        self.schlafgemach.realsense_json_path = self._config['data']['realsense_json']

        if 'addon' in self._config:
            self.schlafgemach.addons = [key for key in self._config['addon']
                                        if self._config['addon'].getboolean(key)]
            for module in self.schlafgemach.addons:
                func = self.schlafgemach.get_init(module)
                self.schlafgemach.addon_init[module] = func()

        print(self.schlafgemach.addons)


    def start(self):
        """ Hier werden alle notwenidigen Processe gestartet und der netzwerkklasse werden alle queues übergeben. """

        q_list = []
        p_list = []

        # Flugsteuerung
        if self._config['device']['moving'] == 'True':
            import cerebellum           # Flugsteuerung

            self.check_thalamus()

            self.kleinhirn = cerebellum.Cerebellum()
            self.kleinhirn.third_dimension = True if self._config['device']['movement'] == '0' else False
            self.kleinhirn.headless = True if self._config['device']['headless'] == 'True' else False
            queue = mp.Queue()
            q_list.append(queue)
            process = mp.Process(target=self.kleinhirn.run, args=(self.schlafgemach, queue))
            p_list.append(process)
            process.start()

        # Bilderkennung
        if self._config['device']['type'] == '0':

            import lobus_occipitalis    # Bild erkennung

            self.sehrinde = lobus_occipitalis.Lobus_occipitalis()
            queue = mp.Queue()
            q_list.append(queue)
            process = mp.Process(target=self.sehrinde.run, args=(self.schlafgemach, queue))
            p_list.append(process)
            process.start()

        # Network

        import corpuscallosum           # Netzwerk interface

        self.balken = corpuscallosum.CorpusCallosum()
        queue = mp.Queue()
        q_list.append(queue)
        process = mp.Process(target=self.balken.run, args=q_list)
        p_list.append(process)
        # process.start()


hirnstamm = TruncusCerebri()
hirnstamm.setup()
hirnstamm.start()
