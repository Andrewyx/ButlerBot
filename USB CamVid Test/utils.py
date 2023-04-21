import cv2
import numpy as np

def thresholding(img):
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowerWhite = np.array([0, 0, 0])
    upperWhite = np.array([179, 255, 255])
    maskWhite = cv2.inRange(imgHSV, lowerWhite, upperWhite)
    return maskWhite    