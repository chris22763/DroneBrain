# Truncus cerebri oder Hirnstamm
# das wird die Main
# Todo make super job managment

import multiprocessing
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

    def load_config(self):
        parser = configparser.ConfigParser()
        parser.read(self._config_path)
        if self._config:
            self._config = {}
            for key in parser:
                self._config[key] = parser[key]
                for sub_key in parser[key]:
                    self._config[key][sub_key] = parser[key][sub_key]

    def check_thalamus(self):

        self.schlafgemach = thalamus.Thalamus()

        if 'addon' in self._config:
            for key in self._config['addon']:
                self.schlafgemach.addons.append(self._config['addon'][key])
            for module in self.schlafgemach.addons:
                func = self.schlafgemach.get_init(module)
                self.schlafgemach.addon_init[module] = func()

    pass

