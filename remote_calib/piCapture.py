# import the necessary packages
import cv2
import numpy as np
import os
from multiprocessing.pool import ThreadPool
import time
from picamera import PiCamera
import picamera.array


#wichtig !!
#"""


PATH = "./images/"
LEFT_PATH = "./images/onBoardCalib/left/{:06d}.jpg"
RIGHT_PATH = "./images/onBoardCalib/right/{:06d}.jpg"

PiIP = '192.168.2.118'
LEFT_URL = "http://192.168.2.118:8082/"
Right_URL = "http://192.168.2.118:8081/"

found = 0
frameId = 0
frameId0 = 0
frameId1 = 0

pattern_size = (4,11)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
OPTIMIZE_ALPHA = 0.25

TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#TERMINATION_CRITERIA = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER

outputFile = "./calibration/onBoardCalib"

########################################Blob Detector##############################################

# Setup SimpleBlobDetector parameters.
blobParams = cv2.SimpleBlobDetector_Params()

# Change thresholds
blobParams.minThreshold = 8
blobParams.maxThreshold = 255

# Filter by Area.
blobParams.filterByArea = True
blobParams.minArea = 64     # minArea may be adjusted to suit for your experiment
blobParams.maxArea = 2500   # maxArea may be adjusted to suit for your experiment

# Filter by Circularity
blobParams.filterByCircularity = True
blobParams.minCircularity = 0.1

# Filter by Convexity
blobParams.filterByConvexity = True
blobParams.minConvexity = 0.87

# Filter by Inertia
blobParams.filterByInertia = True
blobParams.minInertiaRatio = 0.01

# Create a detector with the parameters
blobDetector = cv2.SimpleBlobDetector_create(blobParams)

###################################################################################################

###################################################################################################

# Original blob coordinates, supposing all blobs are of z-coordinates 0
# And, the distance between every two neighbour blob circle centers is 2 centimetres
# In fact, any number can be used to replace 72.
# Namely, the real size of the circle is pointless while calculating camera calibration parameters.
objp = np.zeros((np.prod(pattern_size), 3), np.float32)

objp[0]  = (0, 0, 0)
objp[1]  = (0, 2, 0)
objp[2]  = (0, 4, 0)
objp[3]  = (0, 6, 0)
objp[4]  = (1, 1, 0)
objp[5]  = (1, 3, 0)
objp[6]  = (1, 5, 0)
objp[7]  = (1, 7, 0)
objp[8]  = (2, 0, 0)
objp[9]  = (2, 2, 0)
objp[10] = (2, 4, 0)
objp[11] = (2, 6, 0)
objp[12] = (3, 1, 0)
objp[13] = (3, 3, 0)
objp[14] = (3, 5, 0)
objp[15] = (3, 7, 0)
objp[16] = (4, 0, 0)
objp[17] = (4, 2, 0)
objp[18] = (4, 4, 0)
objp[19] = (4, 6, 0)
objp[20] = (5, 1, 0)
objp[21] = (5, 3, 0)
objp[22] = (5, 5, 0)
objp[23] = (5, 7, 0)
objp[24] = (6, 0, 0)
objp[25] = (6, 2, 0)
objp[26] = (6, 4, 0)
objp[27] = (6, 6, 0)
objp[28] = (7, 1, 0)
objp[29] = (7, 3, 0)
objp[30] = (7, 5, 0)
objp[31] = (7, 7, 0)
objp[32] = (8, 0, 0)
objp[33] = (8, 2, 0)
objp[34] = (8, 4, 0)
objp[35] = (8, 6, 0)
objp[36] = (9, 1, 0)
objp[37] = (9, 3, 0)
objp[38] = (9, 5, 0)
objp[39] = (9, 7, 0)
objp[40] = (10, 0, 0)
objp[41] = (10, 2, 0)
objp[42] = (10, 4, 0)
objp[43] = (10, 6, 0)

###################################################################################################

# Arrays to store object points and image points from all the images.
objpoints0 = [] # 3d point in real world space
objpoints1 = [] # 3d point in real world space

imgpoints0 = [] # 2d points in image plane.
imgpoints1 = [] # 2d points in image plane.




#"""

#Todo everything

#cap0 = cv2.VideoCapture(LEFT_URL)
#cap1 = cv2.VideoCapture(Right_URL)

 
# initialize the camera and grab a reference to the raw camera capture
pool = ThreadPool(processes=2)

imageSize = (int(2952/4), int(1944/4))

print(imageSize)

camera0 = PiCamera(0)
camera0.resolution = imageSize
camera0.rotation = 90
camera0.framerate = 30

camera1 = PiCamera(1)
camera1.resolution = imageSize
camera1.rotation = 90
camera1.framerate = 30
 
# allow the camera to warmup
time.sleep(0.1)


def captureSerialImage(camera,left,frameId, imageSize):
    print(frameId)
    if left:
        path = LEFT_PATH
    else: 
        path = RIGHT_PATH

    #rawCapture = picamera.array.PiRGBArray(camera, size=imageSize)

    with picamera.array.PiRGBArray(camera, size=imageSize) as output:
        camera.capture(output, 'rgb')
        print('Captured %dx%d image' % (output.array.shape[1], output.array.shape[0]))
        # Construct a numpy array from the stream
        data = np.fromstring(output.getvalue(), dtype=np.uint8)
        # "Decode" the image from the array, preserving colour
        img = cv2.imdecode(data, 1)
        
        cv2.imwrite(path.format(frameId), img)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        keypoints = blobDetector.detect(gray) # Detect blobs.


        # Draw detected blobs as red circles. This helps cv2.findCirclesGrid() .
        im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        im_with_keypoints_gray = cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findCirclesGrid(im_with_keypoints, pattern_size, None, flags = cv2.CALIB_CB_ASYMMETRIC_GRID)   # Find the circle grid

        # clear the stream in preparation for the next frame
        camera.truncate()

        return ret, img, corners, im_with_keypoints, im_with_keypoints_gray

while(True):  # Here, 10 can be changed to whatever number you like to choose
    
    async_result_left = pool.apply_async(captureSerialImage,(camera0,True,frameId,imageSize))
    async_result_right = pool.apply_async(captureSerialImage,(camera1,False,frameId,imageSize))
    
    ret0, img0, corners0, im_with_keypoints0, im_with_keypoints_gray0 = async_result_left.get()
    ret1, img1, corners1, im_with_keypoints1, im_with_keypoints_gray1 = async_result_right.get()

    if (ret0 == True) & (ret1 == True):

        #print("fID 0 " + str(frameId0))
        objpoints0.append(objp)  # Certainly, every loop objp is the same, in 3D.

        corners00 = cv2.cornerSubPix(im_with_keypoints_gray0, corners0, (11,11), (-1,-1), criteria)    # Refines the corner locations.
        imgpoints0.append(corners00)

        # Draw and display the corners.
        im_with_keypoints0 = cv2.drawChessboardCorners(img0, (4,11), corners00, ret0)


        #print("fID 1 " + str(frameId1))
        objpoints1.append(objp)  # Certainly, every loop objp is the same, in 3D.

        corners11 = cv2.cornerSubPix(im_with_keypoints_gray1, corners1, (11, 11), (-1, -1),
                                     criteria)  # Refines the corner locations.
        imgpoints1.append(corners11)

        # Draw and display the corners.
        im_with_keypoints1 = cv2.drawChessboardCorners(img1, (4, 11), corners11, ret1)

        #cv2.imwrite(LEFT_PATH_POINTS.format(frameId0), im_with_keypoints0)
        #cv2.imwrite(RIGHT_PATH_POINTS.format(frameId1), im_with_keypoints1)

        frameId = frameId + 1

    