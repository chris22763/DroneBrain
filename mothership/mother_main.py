import socket
import numpy
import time
import cv2
from numpysocket import NumpySocket

npSocket = NumpySocket()
npSocket.startClient(9999)

# Read until video is completed
while(True):
    # Capture frame-by-frame
    frame = npSocket.recieveNumpy()
    cv2.imshow('Frame', frame)

    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

npSocket.endServer()
print('close connection')

