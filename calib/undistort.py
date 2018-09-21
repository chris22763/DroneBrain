import numpy as np
import cv2
import time
from multiprocessing.pool import ThreadPool
from matplotlib import pyplot as plt

PATH_CALIB = "./calibration/calibration.npz"
PATH_IMG_LEFT = "./calibration//left/{:06d}.jpg"
PATH_IMG_RIGHT = "./calibration/right"

data = np.load(PATH_CALIB, allow_pickle=False)

REMAP_INTERPOLATION = cv2.INTER_LINEAR
DEPTH_VISUALIZATION_SCALE = 2048
#for key in data:
#    print(key)

imageSize = tuple(data['imageSize'])
leftMapX = data['leftMapX']
leftMapY = data['leftMapY']
leftROI = tuple(data['leftROI'])
leftCameraMatrix = data['leftCameraMatrix']
leftDistortionCoefficients = data['leftDistortionCoefficients']
leftRectification = data['leftRectification']
leftProjection = data['leftProjection']
leftnewCameraMatrix = data['LeftnewCameraMatrix']
rightMapX = data['rightMapX']
rightMapY = data['rightMapY']
rightROI = tuple(data['rightROI'])
rightRectification = data['rightRectification']
rightDistortionCoefficients = data['rightDistortionCoefficients']
rightProjection = data['rightProjection']
rightCameraMatrix = data['rightCameraMatrix']
rightnewCameraMatrix = data['RightnewCameraMatrix']
#R = data['R']
#T = data['T']
disparityToDepth = data['dispartityToDepthMap']

frameid = 0

    #cv2.namedWindow("remap", cv2.WINDOW_NORMAL)
    #imS = cv2.resize(remap, (960, 540))  # Resize image
    #cv2.imshow("remap", imS)


LEFT_URL = "http://192.168.2.118:8082/"
Right_URL = "http://192.168.2.118:8081/"
#LEFT_URL = "./calibration/left/"
#Right_URL = "./calibration/right/"

left = cv2.VideoCapture(LEFT_URL)
right = cv2.VideoCapture(Right_URL)


pool = ThreadPool(processes=2)


def calculateImage(img0, img1, matcher):


    dispOut = matcher.compute(img0, img1)  # .astype(np.float32)/16
    dispOut = np.int16(dispOut)

    return dispOut



def grab_img(cap, matrix, distCo, newMat):

    _, frame = cap.retrieve()

    imgOut = cv2.undistort(frame, matrix, distCo, None, newMat)


    imgSize = imgOut.shape[:2]
    imgSize = tuple(int(i / resize) for i in imgSize)

    imgScl = cv2.resize(imgOut, imgSize)

    return imgScl

# SGBM Parameters -----------------
window_size = 3  # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

left_matcher = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=160,  # max_disp has to be dividable by 16 f. E. HH 192, 256
    blockSize=5,
    P1=8 * 3 * window_size ** 2,
    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
    P2=32 * 3 * window_size ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=15,
    speckleWindowSize=0,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)


stereoMatcher = cv2.StereoBM_create()
stereoMatcher.setMinDisparity(0)
stereoMatcher.setNumDisparities(128)
stereoMatcher.setBlockSize(25)
stereoMatcher.setSpeckleRange(512)
stereoMatcher.setSpeckleWindowSize(100)

resize = 3

#imgSize = imageSize.shape[:2]
#imgSize = tuple(int(i / resize) for i in imgSize)

#"""
lastFrame = None

#ToDo Thread the shit out of that lovly piece of Code
#ToDo Fire Proof the Pi and Port!

while(True):


    start = time.time()




    if not left.grab() or not right.grab():
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    _, rightFrame = right.retrieve()
    print("retrieve" + str(time.time() - start))

#    imgL = leftFrame
#    imgR = rightFrame



#    leftFrame = cv2.resize(leftFrame, imgSize)
#    rightFrame = cv2.resize(rightFrame, imgSize)

    imgL0 = cv2.undistort(leftFrame, leftCameraMatrix, leftDistortionCoefficients, None, leftnewCameraMatrix)
    imgR0 = cv2.undistort(rightFrame, leftCameraMatrix, leftDistortionCoefficients, None, leftnewCameraMatrix)

    imgSize = imgL0.shape[:2]
    imgSize = tuple(int(i / resize) for i in imgSize)

    imgL = cv2.resize(imgL0,imgSize)
    imgR = cv2.resize(imgR0, imgSize)

    print("undistort"+ str(time.time()-start))

    #async_Grab_Left = pool.apply_async(grab_img,(left,leftCameraMatrix, leftDistortionCoefficients, leftnewCameraMatrix))
    #async_Grab_Right = pool.apply_async(grab_img,(right, leftCameraMatrix, leftDistortionCoefficients,leftnewCameraMatrix))

    #imgL = async_Grab_Left.get()
    #imgR = async_Grab_Right.get()

    # FILTER Parameters
    lmbda = 80000
    sigma = 1.2
    visual_multiplier = 1.0

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)

    #async_Calc_Left = pool.apply_async(calculateImage,(imgL,imgR,left_matcher))
    #async_Calc_Right = pool.apply_async(calculateImage,(imgR,imgL,right_matcher))

    #displ = async_Calc_Left.get()
    #dispr = async_Calc_Right.get()
    displ = right_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
    displ = np.int16(displ)

    dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
    dispr = np.int16(dispr)


    filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!

    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);

    #if  hasattr(filteredImg, 'shape'):
    #    #filteredImg = wls_filter.filter(filteredImg, imgL, None, lastFrame)
    #    filteredImg = cv2.normalize(src=filteredImg, dst=lastFrame, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
    #lastFrame = disparity

    filteredImg = np.uint8(filteredImg)
    #print("hier")
    print("filter" + str(time.time() - start))
    #imgScl = cv2.resize(filteredImg, (540,960))
    #print(disparityToDepth)
    depth = cv2.reprojectImageTo3D(filteredImg ,disparityToDepth)

    #depth = np.uint16(depth)
    #print(depth[0])
    #plt.imshow(depth, 'gray')
    #plt.show()
    cv2.imshow('disparity Map', filteredImg)
    #cv2.imshow('depth Map', depth)

    #numpy_horizontal = np.vstack((displ, dispr))
    #cv2.imshow('raw',numpy_horizontal)
    #cv2.imshow('links',leftFrame)
    #cv2.imshow('rechts', rightFrame)

    #cv2.imshow('og', depth)
    #cv2.imshow('upscale', imgL)
    cv2.waitKey(1)

    end = time.time()
    print(end-start)

cv2.destroyAllWindows()


"""

while (True):

    #left = cv2.VideoCapture(PATH_IMG_LEFT.format(frameid), cv2.CAP_IMAGES)
    #right = cv2.VideoCapture(PATH_IMG_LEFT.format(frameid), cv2.CAP_IMAGES)

    if not left.grab() or not right.grab():
        print("No more frames")
        break

    _, leftFrame = left.retrieve()

    _, rightFrame = right.retrieve()



    fixedLeft = cv2.undistort(leftFrame, leftCameraMatrix, leftDistortionCoefficients, None, leftnewCameraMatrix)
    #x0, y0, w0, h0 = leftROI
    #fixedLeft = fixedLeft[y0:y0 + h0, x0:x0 + w0]

    fixedRight = cv2.undistort(rightFrame, leftCameraMatrix, leftDistortionCoefficients, None, leftnewCameraMatrix)

    #fixedRight = cv2.undistort(rightFrame, rightCameraMatrix, rightDistortionCoefficients, None, rightnewCameraMatrix)
    #x1, y1, w1, h1 = leftROI
    #fixedRight = fixedRight[y1:y1 + h1, x1:x1 + w1]
    
    grayLeft = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
    grayRight = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)

    depth = stereoMatcher.compute(grayLeft, grayRight)

    #cv2.filterSpeckles(depth, 0, 4000, 128);
    # cv2.namedWindow("left", cv2.WINDOW_NORMAL)
    # img_W_L = cv2.resize(fixedLeft, (540, 960))  # Resize image
    # cv2.imshow("left", img_W_L)
    # cv2.namedWindow("right", cv2.WINDOW_NORMAL)
    # img_W_R = cv2.resize(fixedRight, (540, 960))  # Resize image
    # cv2.imshow("right", img_W_L)
    #cv2.imshow("left", fixedLeft)
    #cv2.imshow("right", fixedRight)
    cv2.imshow('depth', depth)

    cv2.waitKey(1)

    frameid = frameid + 1

#"""
