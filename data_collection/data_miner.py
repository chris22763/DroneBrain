import sys

import logging
import time
import struct
import multiprocessing

# import gps


#### Constant ####
NSIZE =  16
OCIMGX = 848
OCIMGY = 480
NEIMGX = OCIMGX / NSIZE
NEIMGY = OCIMGY / NSIZE

#### init  ####

sys.path.append('/home/up/brain/lib/python3.5/site-packages')
print(sys.path[0])

import cv2
import numpy as np
#### the work ###

def maxpull(img, oldImgSize=(OCIMGX,OCIMGY), newImgSize=(NEIMGX,NEIMGY)):
    cropedimg = np.zeros((NEIMGY, NEIMGX, 4), 'uint8')

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

def init_realsense():
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    return pipeline


def init_bno():

    from Adafruit_BNO055 import BNO055

    bno = BNO055.BNO055(rst=13)

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


def encode_float(f, _output, _flag):
    _output.append(_flag['float_start'])
    packed = struct.pack('!f', f)
    if isinstance(packed[0], float):
        integers = [ord(c) for c in packed]  # inverse => [chr(i) for i in integer]
    else:
        integers = [c for c in packed]
    for i in range(integers.__len__()):
        if i%2 == 0:
            if i+1 <= integers.__len__():
                pt = [integers[i], integers[i+1], 0]
                _output.append(pt)
    
    _output.append(_flag['float_end'])
    return _output


def encode_int(i, _output, _flag):
    _output.append(_flag['int'])
    if i <= 255:
        pt = [i, 0, 0]
    elif i <= 255^2:
        pt = [255, i/255, 0]
    elif i <= 255^3:
        pt = [255, 255, i/(255^2)]

    _output.append(pt)
    return _output


def tupel_to_pixel(data):
    output = []
    _flags = {
        'tupel_start': [0, 0 , 128],
        'tupel_end': [0, 0 , 255],
        'float_start': [128, 0, 255],
        'float_end': [255, 0, 255],
        'int' : [0, 255, 255]
    }

    if isinstance(data, tuple):
        output.append(_flags['tupel_start'])
        for d in data:
            if isinstance(d, int):
                output = encode_int(d, output, _flags)
            elif isinstance(d, float):
                output = encode_float(d, output, _flags)
        output.append(_flags['tupel_end'])
    elif isinstance(data, int):
        output = encode_int(data, output, _flags)
    elif isinstance(data, float):
        output = encode_float(data, output, _flags)

    return output


def append_to_img(img, data):
    img_width = img.__len__()
    data_len = data.__len__()


    if data_len <= img_width:
        for i in range(data_len, img_width):
            data.append([0, 0, 0])
        npdata = np.array(data)
        print(img)
        out = np.append(img, npdata)

    return out


def get_bno_data(_device, data_chooser):
    output = {
        0: _device.read_euler(),             # Orientation as a quaternion:                             x,y,z,w
        1: _device.read_temp(),                    # Sensor temperature in degrees Celsius:                temp_c
        2: _device.read_magnetometer(),            # Magnetometer data (in micro-Teslas):                    x,y,z
        3: _device.read_gyroscope(),            # Gyroscope data (in degrees per second):                s.o.
        4: _device.read_accelerometer(),        # Accelerometer data (in meters per second squared):     s.o.
        5: _device.read_linear_acceleration(),    # Linear acceleration data (i.e. acceleration from movement, not gravity--
                                                # returned in meters per second squared):                s.o.
        6: _device.read_gravity()                # Gravity acceleration data (i.e. acceleration just from gravity--returned
    }                                            # in meters per second squared):                        s.o.

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
            print(('data_choser is in the fromt format'))


def get_realsense_data(pipeline):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    return depth_frame, color_frame


def realsense_to_numpy(frame):

    image = np.asanyarray(frame.get_data())

    return image


def make_image(color_image, depth_image):
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # Stack both images horizontally
    images = np.hstack((color_image, depth_colormap))

    # Show images
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', images)

    cv2.waitKey(1)


bno = init_bno()
pipeline = init_realsense()
try:
    while True:
        data_row = [tupel_to_pixel(get_bno_data(bno, i)) for i in range(6)]

        depth_frame, color_frame = get_realsense_data(pipeline)

        if not depth_frame and not color_frame:
            continue

        depth_image = realsense_to_numpy(depth_frame)
        # color_image = realsense_to_numpy(color_image)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        data_img = append_to_img(depth_colormap, data_row)
        # Stack both images horizontally
        # images = np.hstack((color_image, depth_colormap))

        # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)

        cv2.waitKey(1)

        print(data_row)

        time.sleep(1)

finally:

    # Stop streaming
    pipeline.stop()
