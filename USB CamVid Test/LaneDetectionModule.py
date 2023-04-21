import cv2
import numpy as np
import utils

def getLaneCurve(img):
    imgThres = utils.thresholding(img)
    cv2.imshow('Thres', imgThres)
    return None

if __name__ == '__main__':
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    while True:
        success, img = cap.read()
        img = cv2.resize(img, (480, 240))
        getLaneCurve(img)
        
        cv2.imshow("LiveFeed", img)
        cv2.waitKey(1)
        k = cv2.waitKey(30) & 0xff
        if k==27:
            break
    # Release the VideoCapture object
    cap.release()
    cv2.destroyAllWindows()        