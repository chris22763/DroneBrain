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


    def load_config(self):
        parser = configparser.ConfigParser()
        parser.read(self._config_path)
        if self._config:
            self._config = {}
            for key in parser:
                self._config[key] = parser[key]
                for sub_key in parser[key]:
                    self._config[key][sub_key] = parser[key][sub_key]


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

        if 'addon' in self._config:
            for key in self._config['addon']:
                self.schlafgemach.addons.append(self._config['addon'][key])
            for module in self.schlafgemach.addons:
                func = self.schlafgemach.get_init(module)
                self.schlafgemach.addon_init[module] = func()


    def start(self):
        """ Hier werden alle notwenidigen Processe gestartet und der netzwerkklasse werden alle queues übergeben. """
        
        q_list = []
        p_list = []

        # Flugsteuerung
        if self._config['device']['moving'] == 'True':

            import cerebellum           # Flugsteuerung

            self.kleinhirn = cerebellum.Cerebellum()
            self.kleinhirn.third_dimension = True if self._config['device']['movement'] == '0' else False

            self.check_thalamus(self)

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

        self.balken = corpuscallosum.Corpus_callosum()
        queue = mp.Queue()
        q_list.append(queue)
        process = mp.Process(target=self.balken.run, args=q_list)
        p_list.append(process)
        process.start()



hirnstamm = TruncusCerebri()
hirnstamm._config_path = "/data/dummy_drone_config.cfg"
hirnstamm.load_config()
print(hirnstamm._config)
hirnstamm.start()