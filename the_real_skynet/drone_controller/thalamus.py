import sys

import logging
import time
import struct
import multiprocessing
import traceback

sys.path.append('/home/up/brain/lib/python3.5/site-packages')

import cv2
import numpy as np

class Thalamus:

    def __init__(self):
        self._search_array = []
        self.type = 0           # 0 = Def., 1 = MainPc, 2 = Drone               # which part does the instance take
        self.movable = False    # True if could be moving
        self.addons = []         # 'GPS', 'BNO', 'realsense', 'Wifi' (-repeater) # Makes ist possible to check witch data is accsesible
        self.addon_init = {}    # Stores the objects, with a key (e.g. 'GPS' : gps_session), created to recieve the sensor data.
        self.sensor_data = {}
        self.scale = 3
        self.realsense_json_path = ''


    def find_device_that_supports_advanced_mode() :
        ctx = rs.context()
        ds5_dev = rs.device()
        devices = ctx.query_devices();
        for dev in devices:
            if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                if dev.supports(rs.camera_info.name):
                    print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
                return dev
        raise Exception("No device that supports advanced mode was found")


    def create_chunk(self, x, y, x_max, y_max, data=None):
        chunk = []
        lim_x = x + self.scale if x + self.scale >= x_max else x_max
        lim_y = y + self.scale if y + self.scale >= y_max else y_max
        if data:

            for _y in range(y, lim_y):
                for _x in range(x, lim_x):
                    chunk.append(data[_x][_y])
            return chunk

        else:
            for _y in range(y, lim_y):
                for _x in range(x, lim_x):
                    chunk.append((_x, _y))
            return chunk


    def create_flower(self, xs, ys, num_pts=1000):

        from numpy import pi, cos, sin, sqrt, arange

        # xs = self.img_size[0]
        # ys = self.img_size[1]
        hx = int(xs/2)
        hy = int(ys/2)
        indices = arange(0, num_pts, dtype=float) + 0.5

        r = sqrt(indices/num_pts)
        theta = pi * (1 + 5**0.5) * indices

        flower = [[int(x+hx),int(y+hy)] for x, y in zip((r*cos(theta)*hx), (r*sin(theta)*hy))]

        return flower


    def create_search_spiral(self, X, Y):
        _X = int(X / self.scale)
        _Y = int(Y / self.scale)
        _search_array = []
        x = y = 0
        dx = 0
        dy = -1
        cx = int(_X/2)  # center output to middle of image
        cy = int(_Y/2)
        for i in range(max(_X, _Y)**2):
            if (-_X/2 < x <= _X/2) and (-_Y/2 < y <= _Y/2):

                print (x, y)
                _search_array.append(((x + cx) * self.scale, (y + cy) * self.scale))

            if x == y or (x < 0 and x == -y) or (x > 0 and x == 1-y):
                dx, dy = -dy, dx
            x, y = x + dx, y + dy

        self._search_array = _search_array
        return self._search_array.append


    def get_init(self, key):
        func = {
            'gps': self.init_gps,
            'bno': self.init_bno,
            'realsense': self.init_realsense,
            'wifi': self.init_wifi
        }
        return func[key]


    def init_realsense(self):
        import pyrealsense2 as rs
        import json
        print('initiate Realsense')

        def find_device_that_supports_advanced_mode() :
            DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07","0B3A"]

            ctx = rs.context()
            ds5_dev = rs.device()
            devices = ctx.query_devices();
            for dev in devices:
                if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                    if dev.supports(rs.camera_info.name):
                        print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
                    return dev
            raise Exception("No device that supports advanced mode was found")

        try:

            dev = find_device_that_supports_advanced_mode()
            advnc_mode = rs.rs400_advanced_mode(dev)
            print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

            while not advnc_mode.is_enabled():
                print("Trying to enable advanced mode...")
                advnc_mode.toggle_advanced_mode(True)
                # At this point the device will disconnect and re-connect.
                print("Sleeping for 5 seconds...")
                time.sleep(5)
                # The 'dev' object will become invalid and we need to initialize it again
                dev = find_device_that_supports_advanced_mode()
                advnc_mode = rs.rs400_advanced_mode(dev)
                print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

            print_config = False
            if print_config:
                pass
                # serialized_string = advnc_mode.serialize_json()
                # print("Controls as JSON: \n", serialized_string)


            if self.realsense_json_path:
                as_json_object = None
                with open(self.realsense_json_path) as jc:
                    as_json_object = json.load(jc)
                if type(next(iter(as_json_object))) != str:
                    as_json_object = {k.encode('utf-8'): v.encode("utf-8") for k, v in as_json_object.items()}

                json_string = str(as_json_object).replace("'", '\"')
                advnc_mode.load_json(json_string)
                # serialized_string = advnc_mode.serialize_json()
                # print("Controls as JSON: \n", serialized_string)

            max_x = 848
            max_y = 480
            # max_x = 424
            # max_y = 240

            self.resolution = (max_y, max_x)

            # todo custom config
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, max_x, max_y, rs.format.z16, 90)

            config.enable_stream(rs.stream.color, max_x, max_y, rs.format.bgr8, 30)

            # Start streaming
            # pipeline.start(config)
            self.addon_init['realsense'] = (pipeline, config)

            return (pipeline, config)

        except Exception as e:
            print(traceback.print_exc())


    def init_bno(self):
        from Adafruit_BNO055 import BNO055
        print('initiate BNO')

        bno = BNO055.BNO055(rst=13)

        if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
            logging.basicConfig(level=logging.DEBUG)

        # Initialize the BNO055 and stop if something went wrong.
        if not bno.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
        """
        # Print system status and self test result.
        status, self_test, error = bno.get_system_status()
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        # Print out an error if system status is in error mode.
        if status == 0x01:
            print('System error: {0}'.format(error))
            print('See datasheet section 4.3.59 for the meaning.')
    
        # Print BNO055 software revision and other diagnostic data.
        sw, bl, accel, mag, gyro = bno.get_revision()
        print('Software version:   {0}'.format(sw))
        print('Bootloader version: {0}'.format(bl))
        print('Accelerometer ID:   0x{0:02X}'.format(accel))
        print('Magnetometer ID:    0x{0:02X}'.format(mag))
        print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
    
        print('Reading BNO055 data, press Ctrl-C to quit...')
        
        """

        self.addon_init['bno'] = bno
        return bno


    def init_gps(self):
        import gps

        print('initiate GPS')

        # from gps3 import gps3
        # gps_socket = gps3.GPSDSocket()
        # data_stream = gps3.DataStream()
        # gps_socket.connect()
        # gps_socket.watch()
        # for new_data in gps_socket:
        #     if new_data:
        #         data_stream.unpack(new_data)
        #
        #         print('Altitude = ', data_stream.TPV['alt'])
        #         print('Latitude = ', data_stream.TPV['lat'])
        # Listen on port 2947 of gpsd

        session = gps.gps("localhost", "2947")
        session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        self.get_gps(session)
        print(session)
        return session


    def init_wifi(self):
        print('initiate WIFI')
        pass


    def get_bno_data(self, _device, data_chooser, _list):
        key = {
            0: 'eulerAngle',
            1: 'temprature',
            2: 'magnetometer',
            3: 'gyroscope',
            4: 'accelerometer',
            5: 'linear_acceleration',
            6: 'gravity'
        }
        output = {
            # for some reason not all "read" methods work
            0: _device.read_euler(),               # Orientation as a quaternion:                             x,y,z,w
            1: _device.read_temp(),                # Sensor temperature in degrees Celsius:                temp_c
            2: _device.read_magnetometer(),        # Magnetometer data (in micro-Teslas):                    x,y,z
            3: _device.read_gyroscope(),           # Gyroscope data (in degrees per second):                s.o.
            4: _device.read_accelerometer(),       # Accelerometer data (in meters per second squared):     s.o.
            5: _device.read_linear_acceleration(), # Linear acceleration data (i.e. acceleration from movement, not gravity--
            # returned in meters per second squared):                s.o.
            6: _device.read_gravity()              # Gravity acceleration data (i.e. acceleration just from gravity--returned
        }                                          # in meters per second squared):                        s.o.

        # this block makes it possible to choose the needed data

        if isinstance(data_chooser, int):
            _data = output[data_chooser]
            self.sensor_data[key[data_chooser]] = _data

        elif isinstance(data_chooser, str):
            try:
                index = int(data_chooser)
                _data = output[index]
                self.sensor_data[key[index]] = _data
            except:
                print('data_choser is in the wrong format')
        else:
            try:
                _data = []
                for entry in data_chooser:
                    _data = output[entry]
                    self.sensor_data[key[entry]] = _data
            except:
                print('data_choser is in the wrong format')


    def get_gps(self, session):
        while True:
            try :
                rep = session.next()
                if rep["class"] == "TPV":
                    print(str(rep.lat) + "," + str(rep.lon))
                    self.sensor_data['GPS'] = [rep.lat, rep.lon]
                    return [rep.lat, rep.lon]

            except Exception as e :
                print("Got exception " + str(e))
                # TODO remove this
                print(rep)
                self.sensor_data['GPS'] = [0.0, 0.0]
                return [0.0, 0.0]


    def get_realsense_data(self, pipe_package):
        pipeline = pipe_package[0]
        config = pipe_package[1]
        while True:
            try:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                # color_frame = frames.get_color_frame()

                if depth_frame is not None:
                    break

            except Exception as e:
                try:
                    pipeline.stop()
                except:
                    pipeline.start(config)

                print(e)
        return depth_frame


    def realsense_to_numpy(self, frame):
        # convert the realsense img to a numpy array readable by opencv
        image = np.asanyarray(frame.get_data())
        return image


