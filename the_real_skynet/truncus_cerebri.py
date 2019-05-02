# Truncus cerebri oder Hirnstamm
# das wird die Main
# Todo make super job managment

import multiprocessing as mp
import sys
import configparser
import thalamus             # Alle Sensoren
import cerebellum           # Flugsteuerung
import corpus_callosum      # Netzwerk interface
import lobus_occipitalis    # Bild erkennung


class Truncus_cerebri():

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
        output = {
            'network': None,
            'command': None,
        }
        if data:
            if key == 'network':
                # example data
                output[key] = {
                    dev_name: data[0],  # 'drone_nr_00', 
                    route: data[1],     # '192.168.2.1,192.168.2.2,192.168.2.3',
                }

            elif key == 'command':
                pass

            elif key == 'network':
                pass
            
            for q in queue:
                q.put(output)    

    def check_thalamus(self):

        self.schlafgemach = thalamus.Thalamus()

        if 'addon' in self._config:
            for key in self._config['addon']:
                self.schlafgemach.addons.append(self._config['addon'][key])
            for module in self.schlafgemach.addons:
                func = self.schlafgemach.get_init(module)
                self.schlafgemach.addon_init[module] = func()


    def start(self):
        q_list = []
        p_list = []

        # Flugsteuerung
        if self._config['type']['movement'] != 'none':
            self.kleinhirn = cerebellum.Cerebellum()
            queue = mp.Queue()
            q_list.append(queue)
            process = mp.Process(target=self.kleinhirn.run, args=(self.schlafgemach, queue))
            p_list.append(process)
            process.start()

        # Bilderkennung
        if self._config['type'] == 'Mainframe':
            self.sehrinde = lobus_occipitalis.Lobus_occipitalis()
            queue = mp.Queue()
            q_list.append(queue)
            process = mp.Process(target=self.sehrinde.run, args=(self.schlafgemach, queue))
            p_list.append(process)
            process.start()

        # Network
        self.balken = corpus_callosum.Corpus_callosum()
        queue = mp.Queue()
        q_list.append(queue)
        process = mp.Process(target=self.balken.run, args=q_list)
        p_list.append(process)
        process.start()



hirnstamm = Truncus_cerebri()

hirnstamm.start()
