import Adafruit_BNO055 as gpio
import pyrealsense2 as rs
import cv2
import numpy as np

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

def getCompass(bno055Object):
	offset (0,0,0)
	data (0,0,0)
	return data + offset

