import data_miner as steve
import sys
import time

sys.path.append('/home/up/brain/lib/python3.5/site-packages')

from numpysocket import NumpySocket
import cv2
import numpy as np


def avoid_Obstacles(img):



def main():
    try:
        bno = steve.init_bno()
        pipeline = steve.init_realsense()


        host_ip = '192.168.2.102'  # change me
        npSocket = NumpySocket()
        npSocket.startServer(host_ip, 9999)

        while True:
            for i in range(6):
                data_groupe = steve.get_bno_data(bno, i, data_groupe)
            depth_frame = steve.get_realsense_data(pipeline)

            if not depth_frame:
                continue

            depth_image = steve.realsense_to_numpy(depth_frame)

            crop_img = steve.maxpull(depth_image)

            npSocket.sendNumpy(crop_img)

            cv2.waitKey(0.1)
            time.sleep(0.1)

    finally:

        # Stop streaming
        pipeline.stop()
        npSocket.endServer()


main()