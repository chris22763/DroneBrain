import pyrealsense2 as rs
import cv2
import numpy as np
import logging
import sys
import time

from Adafruit_BNO055 import BNO055

import multiprocessing

# import gps


#### Constant ####
NSIZE =  16
OCIMGX = 848
OCIMGY = 480
NEIMGX = OCIMGX / NSIZE
NEIMGY = OCIMGY / NSIZE

#### init  ####


#### the work ###

def maxpull(img, oldImgSize=(OCIMGX,OCIMGY), newImgSize=(NEIMGX,NEIMGY)):
    for x in range(newImgSize[0]):
        for y in range(newImgSize[1]):
            x0 = (x * NSIZE)
            y0 = (y * NSIZE)

            x1 = (x * NSIZE) + (NSIZE-1)
            y1 = (y * NSIZE) + (NSIZE-1)

            mask = img[y0:y1, x0:x1]
            minMax = cv2.minMaxLoc(mask)
            val = 1 / minMax[1]

            cropedimg[y][x] = val
    return cropedimg

def init_bno():
    bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

    if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
        logging.basicConfig(level=logging.DEBUG)

    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

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

    return bno

def get_bno_data(_device, data_chooser):
    output = {
        0: _device.read_quaterion(), 			# Orientation as a quaternion: 							x,y,z,w
        1: _device.read_temp(),					# Sensor temperature in degrees Celsius:				temp_c
        2: _device.read_magnetometer(),			# Magnetometer data (in micro-Teslas):					x,y,z
        3: _device.read_gyroscope(),			# Gyroscope data (in degrees per second):				s.o.
        4: _device.read_accelerometer(),		# Accelerometer data (in meters per second squared): 	s.o.
        5: _device.read_linear_acceleration(),	# Linear acceleration data (i.e. acceleration from movement, not gravity--
                                                # returned in meters per second squared):				s.o.
        6: _device.read_gravity()				# Gravity acceleration data (i.e. acceleration just from gravity--returned
    }											# in meters per second squared):						s.o.

    if isinstance(data_chooser, int):
        _data = output[data_chooser]
        return _data

    elif isinstance(data_chooser, str):
        try:
            index = int(data_chooser)
            _data = output[index]
            return _data
        except:
            print('data_choser is in the fromt format')
    else:
        try:
            _data = []
            for entry in data_chooser:
                _data.append(output[entry])
            return _data
        except:
            print(('data_choser is in the fromt format')


bno = init_bno()

bno_data = get_bno_data(bno, 0)

print(bno_data)
while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(heading, roll, pitch, sys, gyro, accel, mag))
    # Other values you can optionally read:
    # Orientation as a quaternion:
    #x,y,z,w = bno.read_quaterion()
    # Sensor temperature in degrees Celsius:
    #temp_c = bno.read_temp()
    # Magnetometer data (in micro-Teslas):
    #x,y,z = bno.read_magnetometer()
    # Gyroscope data (in degrees per second):
    #x,y,z = bno.read_gyroscope()
    # Accelerometer data (in meters per second squared):
    #x,y,z = bno.read_accelerometer()
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #x,y,z = bno.read_linear_acceleration()
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()
    # Sleep for a second until the next reading.
    time.sleep(1)