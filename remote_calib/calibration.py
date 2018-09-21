# import the necessary packages
import cv2
import numpy as np
import os
from multiprocessing.pool import ThreadPool
import pprint
import time



PATH = "./images/"
LEFT_PATH = "./images/left/{:06d}.jpg"
RIGHT_PATH = "./images/right/{:06d}.jpg"

PiIP = '192.168.2.118'
LEFT_URL = "http://192.168.2.118:8082/"
Right_URL = "http://192.168.2.118:8081/"
frameId0 = 0
frameId1 = 0

pattern_size = (4,11)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
OPTIMIZE_ALPHA = 0.25

TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#TERMINATION_CRITERIA = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER

outputFile = "./images/onBoardCalib"

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


found = 0
imageSize = (0,0)
"""

while(True):
    # grb_img0 = cap0.grab()
    # grb_img1 = cap1.grab()
    #
    ret0, frame0 = cap0.retrieve()
    ret1, frame1 = cap1.retrieve()

    #ret_img0 = cap0.read()
    #ret_img1 = cap1.read()


    status0, centers0 = cv2.findCirclesGrid(frame0, pattern_size, cv2.CALIB_CB_ASYMMETRIC_GRID)
    status1, centers0 = cv2.findCirclesGrid(frame1, pattern_size, cv2.CALIB_CB_ASYMMETRIC_GRID)

    if status0:
        print(str(status0) + " :no image found Left : " + str(frameId0))
        cv2.imwrite(LEFT_PATH.format(frameId0), frame0)
        frameId0 = frameId0 + 1
    else:
        print(str(status0) + " :no image found Left : " + str(frameId0))
        cv2.imwrite(LEFT_PATH.format(frameId0), frame0)
        frameId0 = frameId0 + 1

    if status1:
        print(str(status0) + " :no image found Right : " + str(frameId1))
        cv2.imwrite(RIGHT_PATH.format(frameId1), frame1)
        frameId1 = frameId1 + 1
    else:
        print(str(status0) + " :no image found Right : " + str(frameId1))
        cv2.imwrite(RIGHT_PATH.format(frameId1), frame1)
        frameId1 = frameId1 + 1

    #if status0 == True:
        #objpoints.append(objp)
    #    cv2.cornerSubPix(frame0, centers0, (11, 11), (-1, -1), criteria)
    #    imgpoints.append(centers0)
    #    cv2.drawChessboardCorners(frame0, (7, 6), centers0, status0)
    #    cv2.imshow('img', frame0)
    #    cv2.waitKey(500)
    #cv2.imshow('frame0', frame0)
    #cv2.imshow('frame1',frame1)




    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(1)
"""


def distortion_matrix(path, objpoints, imgpoints, imageSize):
    found = 0
    start = time.time()
    cap = cv2.VideoCapture(path + "000000.jpg",cv2.CAP_IMAGES)


    for item in os.listdir(path):

        if item.endswith(".jpg"):
            cap = cv2.VideoCapture(path+item, cv2.CAP_IMAGES)

            ret, img = cap.read()  # Capture frame-by-frame
            if imageSize == (0,0):
                imageSize = img.shape[:2]

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            keypoints = blobDetector.detect(gray)  # Detect blobs.

            # Draw detected blobs as red circles. This helps cv2.findCirclesGrid() .
            im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0, 255, 0),
                                                  cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            im_with_keypoints_gray = cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findCirclesGrid(im_with_keypoints, (4, 11), None,
                                               flags=cv2.CALIB_CB_ASYMMETRIC_GRID)  # Find the circle grid

            if ret == True:
                print( path + " " + item + " " + str(found))
                objpoints.append(objp)  # Certainly, every loop objp is the same, in 3D.

                corners2 = cv2.cornerSubPix(im_with_keypoints_gray, corners, (11, 11), (-1, -1),
                                            criteria)  # Refines the corner locations.
                imgpoints.append(corners2)
                found = found + 1


    cap.release()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    h, w = imageSize
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    end = time.time()
    print(path + str(end-start))
    return (ret, mtx, dist, rvecs, tvecs, objpoints, imgpoints, imageSize, roi, newcameramtx)

pool = ThreadPool(processes=2)

async_result_left = pool.apply_async(distortion_matrix, ("./calibration/left/", objpoints0, imgpoints0, imageSize))
async_result_right =  pool.apply_async(distortion_matrix, ("./calibration/right/", objpoints1, imgpoints1, imageSize))

_, leftCameraMatrix, leftDistortionCoefficients, _, _ , objpoints0, imgpoints0, imageSize, leftROI, newcameramtx0 = async_result_left.get()
_, rightCameraMatrix, rightDistortionCoefficients, _, _, objpoints1, imgpoints1, imageSize, rightROI, newcameramtx1 = async_result_right.get()

"""

cap0 = cv2.VideoCapture(LEFT_URL)
cap1 = cv2.VideoCapture(Right_URL)

#(frameId0 < 20) & (frameId1 < 20)

while(True ):  # Here, 10 can be changed to whatever number you like to choose
    #ret0, img0 = cap0.read() # Capture frame-by-frame
    #ret1, img1 = cap1.read()  # Capture frame-by-frame

    if not (cap0.grab() and cap1.grab()):
        print("No more frames")
        break

    _, img0 = cap0.retrieve()
    _, img1 = cap1.retrieve()

    cv2.imwrite(LEFT_PATH.format(frameId0), img0)
    cv2.imwrite(RIGHT_PATH.format(frameId1), img1)

    imageSize = img0.shape[:2]

    gray0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)

    keypoints0 = blobDetector.detect(gray0) # Detect blobs.
    keypoints1 = blobDetector.detect(gray1)  # Detect blobs.

    # Draw detected blobs as red circles. This helps cv2.findCirclesGrid() .
    im_with_keypoints0 = cv2.drawKeypoints(img0, keypoints0, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    im_with_keypoints_gray0 = cv2.cvtColor(im_with_keypoints0, cv2.COLOR_BGR2GRAY)
    ret0, corners0 = cv2.findCirclesGrid(im_with_keypoints0, pattern_size, None, flags = cv2.CALIB_CB_ASYMMETRIC_GRID)   # Find the circle grid

    im_with_keypoints1 = cv2.drawKeypoints(img1, keypoints1, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    im_with_keypoints_gray1 = cv2.cvtColor(im_with_keypoints1, cv2.COLOR_BGR2GRAY)
    ret1, corners1 = cv2.findCirclesGrid(im_with_keypoints1, pattern_size, None, flags = cv2.CALIB_CB_ASYMMETRIC_GRID)   # Find the circle grid

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

        cv2.imwrite(LEFT_PATH_POINTS.format(frameId0), im_with_keypoints0)
        cv2.imwrite(RIGHT_PATH_POINTS.format(frameId1), im_with_keypoints1)

        frameId0 = frameId0 + 1
        frameId1 = frameId1 + 1

    #input("Press Enter to continue...")




    #cv2.imshow("img0", im_with_keypoints0) # display
        

print (" imgp0 :"+str(imgpoints0.__len__()) + " opjp0 :" + str(objpoints0.__len__()))
print (" imgp1 :"+str(imgpoints1.__len__()) + " opjp1 :" + str(objpoints1.__len__()))

# When everything done, release the capture        
cap0.release()
cap1.release()


# cam = cv.VideoCapture("./imageSequences/small/img_%05d.jpg", cv.CAP_IMAGES)

_, leftCameraMatrix, leftDistortionCoefficients, _, _ = cv2.calibrateCamera(objpoints0, imgpoints0, gray0.shape[::-1], None, None)
_, rightCameraMatrix, rightDistortionCoefficients, _, _ = cv2.calibrateCamera(objpoints1, imgpoints1, gray1.shape[::-1], None, None)

#"""

#_, leftCameraMatrix, leftDistortionCoefficients, _, _ , objpoints0, imgpoints0, imageSize, leftROI, newcameramtx0 = _thread.start_new_thread(distortion_matrix, ("./calibration/left/", objpoints0, imgpoints0, imageSize))
#_, rightCameraMatrix, rightDistortionCoefficients, _, _, objpoints1, imgpoints1, imageSize, rightROI, newcameramtx1 = _thread.start_new_thread(distortion_matrix, ("./calibration/right/", objpoints1, imgpoints1, imageSize))

#_, leftCameraMatrix, leftDistortionCoefficients, _, _ , objpoints0, imgpoints0, imageSize, leftROI, newcameramtx0 = distortion_matrix("./calibration/left/", objpoints0, imgpoints0, imageSize)

#_, rightCameraMatrix, rightDistortionCoefficients, _, _, objpoints1, imgpoints1, imageSize, rightROI, newcameramtx1 = distortion_matrix("./calibration/right/", objpoints1, imgpoints1, imageSize)

"""
#ret0, mtx0, dist0, rvecs0, tvecs0 = distortion_matrix("./calibration/left/%06d.jpg")
#ret1, mtx1, dist1, rvecs1, tvecs1 = distortion_matrix("./calibration/right/%06d.jpg")


img0 = cv2.imread('./calibration/left/000012.jpg')
h0,  w0 = img0.shape[:2]
newcameramtx0, roi0=cv2.getOptimalNewCameraMatrix(mtx0,dist0,(w0,h0),1,(w0,h0))

# undistort
mapx0,mapy0 = cv2.initUndistortRectifyMap(mtx0,dist0,None,newcameramtx0,(w0,h0),5)
dst0 = cv2.remap(img0,mapx0,mapy0,cv2.INTER_LINEAR)

# crop the image
x0,y0,w0,h0 = roi0
dst0 = dst0[y0:y0+h0, x0:x0+w0]
cv2.imwrite('./calibration/calibresult_left.png',dst0)

img1 = cv2.imread('./calibration/right/000012.jpg')
h1,  w1 = img1.shape[:2]
newcameramtx1, roi1=cv2.getOptimalNewCameraMatrix(mtx1,dist1,(w1,h1),1,(w1,h1))

# undistort
mapx1,mapy1 = cv2.initUndistortRectifyMap(mtx1,dist1,None,newcameramtx1,(w1,h1),5)
dst1 = cv2.remap(img1,mapx1,mapy1,cv2.INTER_LINEAR)

# crop the image
x1,y1,w1,h1 = roi1
dst1 = dst1[y1:y1+h1, x1:x1+w1]
cv2.imwrite('./calibration/calibresult_right.png',dst1)
"""
print(str(imgpoints0.__len__()) + " " + str(imgpoints1.__len__()) + " " + str(objpoints0.__len__()) + " " + str(objpoints1.__len__()))
NUM_IMAGES = imgpoints0.__len__()
objp = np.repeat(objp[np.newaxis, :, :], NUM_IMAGES, axis=0)

(_, _, _, _, _, rotationMatrix, translationVector, _, _) = cv2.stereoCalibrate( objp, imgpoints0, imgpoints1,
                                                                                leftCameraMatrix, leftDistortionCoefficients,
                                                                                rightCameraMatrix, rightDistortionCoefficients,
                                                                                imageSize, None, None, None, None,
                                                                                cv2.CALIB_FIX_INTRINSIC, TERMINATION_CRITERIA )


print("Rectifying cameras...")
# TODO: Why do I care about the disparityToDepthMap?
# cv2.stereoRectify gibt falsche ROI werte aus, darum ermittel ich in distortion_matrix
# diese über einen umweg
(leftRectification, rightRectification, leftProjection, rightProjection,
        dispartityToDepthMap, _ , _) = cv2.stereoRectify( leftCameraMatrix, leftDistortionCoefficients, rightCameraMatrix, rightDistortionCoefficients,
                imageSize, rotationMatrix, translationVector, None, None, None, None, None,
                cv2.CALIB_ZERO_DISPARITY, OPTIMIZE_ALPHA)

print(dispartityToDepthMap)

print("Saving calibration...")
leftMapX, leftMapY = cv2.initUndistortRectifyMap(
        leftCameraMatrix, leftDistortionCoefficients, leftRectification,
        leftProjection, imageSize, cv2.CV_32FC1)
rightMapX, rightMapY = cv2.initUndistortRectifyMap(
        rightCameraMatrix, rightDistortionCoefficients, rightRectification,
        rightProjection, imageSize, cv2.CV_32FC1)


np.savez_compressed(outputFile, imageSize=imageSize,
        leftMapX=leftMapX, leftMapY=leftMapY, leftROI=leftROI, leftCameraMatrix = leftCameraMatrix,
        leftDistortionCoefficients = leftDistortionCoefficients, leftRectification = leftRectification,
        leftProjection = leftProjection, LeftnewCameraMatrix = newcameramtx0,
        rightMapX=rightMapX, rightMapY=rightMapY, rightROI=rightROI, rightRectification = rightRectification,
        rightDistortionCoefficients = rightDistortionCoefficients, rightCameraMatrix = rightCameraMatrix,
        RightnewCameraMatrix = newcameramtx1, rightProjection = rightProjection,
        R = rotationMatrix , T = translationVector, dispartityToDepthMap = dispartityToDepthMap)

cv2.destroyAllWindows()