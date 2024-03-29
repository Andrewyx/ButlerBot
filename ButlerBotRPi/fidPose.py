import numpy as np
import cv2
import sys
import time
import math
import serial
import json

data = {"id": 0, "distance": 0, "angle": 0, "isoffcourse": False, "RVec": 0, "Reorientate": False}
serialCounter = 0
serialCounterForBounding = 0
isBoundedbyBox = False

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

screenWidth = 640
screenHeight = 480
screenCenter = (screenWidth / 2, screenHeight / 2)
colour = (0, 0, 255)
thickness = 8
beenSent = False
ser = serial.Serial("/dev/ttyS0", 9600)

def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:
        ids = ids.flatten()

        for markerCorner, markerID in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            startpoint = (screenWidth/2, screenHeight/2)
            endpoint = (cX, cY)
            cv2.line(image, startpoint, endpoint, (0, 255, 0), thickness)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            

            cv2.putText(
                image,
                str(markerID),
                (topLeft[0], topLeft[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            print("[Inference] ArUco marker ID: {}".format(markerID))

    return image

centerPixel = None

def vector_from_center(screenCenter, fidCenter):
    fidRelative = tuple(map(lambda i, j: i - j, fidCenter, screenCenter))
    distanceInPx = math.sqrt(fidRelative[0]**2 + fidRelative[1]**2)
    angleInRad = math.atan2(fidRelative[1], fidRelative[0])
    fidVec = [distanceInPx, angleInRad]
    global data
    data["distance"] = float(distanceInPx)
    data["angle"] = float(angleInRad) 
    data["isoffcourse"] = True;   
    return fidVec

def sendThroughSerial(ids, fidDetected):
    global data, serialCounter, beenSent
    if fidDetected is False and beenSent is False:
        data["id"] = 0
        beenSent = True
        Jdata = json.dumps(data)
        ser.write(Jdata.encode('ascii'))
        ser.write(b"\n")
        ser.flush()        
        return print(Jdata) 
    elif fidDetected is True:
        idlist = ids.tolist()
        data["id"] = idlist[0][0]
        Jdata = json.dumps(data)
        beenSent = False
        #print(Jdata) 
        if serialCounter >= 10:
            ser.write(Jdata.encode('ascii'))
            ser.write(b"\n")
            ser.flush()
            serialCounter = 0
        else:
            serialCounter+=1
               

def isInsideBox(fidCenterPx, lowerx, upperx, lowery, uppery):
    if fidCenterPx[0] >= lowerx and fidCenterPx[0] <= upperx:
        if fidCenterPx[1] >= lowery and fidCenterPx[1] <= uppery:
            global isBoundedbyBox 
            isBoundedbyBox = True
            return True
    return False

def boundingBox(frame, boxlength, boxheight, colour, lineThickness, fidCenterPx, rvec, i):
    global screenWidth, screenHeight, data
    xLeftCornerPx = int((screenWidth - boxlength)/2)
    yLeftCornerPx = int((screenHeight - boxheight)/2)
    leftRectCorner = (xLeftCornerPx, yLeftCornerPx)
    rightRectCorner = (screenWidth - xLeftCornerPx, screenHeight - yLeftCornerPx)
    data["Reorientate"] = False
    if fidCenterPx is None:
        pass
    elif isInsideBox(fidCenterPx, leftRectCorner[0], rightRectCorner[0], leftRectCorner[1], rightRectCorner[1]):
        colour = (0,0,255)
        correctOrientationData(rvec, i)
        
        
    frame_with_rect = cv2.rectangle(frame, leftRectCorner, rightRectCorner ,colour,lineThickness)

    return frame_with_rect

def correctOrientationData(rvec, i):
    global data, serialCounterForBounding
    rotM = np.zeros(shape=(3,3))
    cv2.Rodrigues(rvec[i-1], rotM, jacobian = 0)
    ypr = cv2.RQDecomp3x3(rotM)

    data["RVec"] = ypr[0][2]
    data["Reorientate"] =  True
    Jdata = json.dumps(data)
    if serialCounterForBounding >= 5:
        ser.write(Jdata.encode('ascii'))
        ser.write(b"\n")
        ser.flush()       
    else:
        serialCounterForBounding += 1            
    #we want roll, the third item
    #rotate bot until the Euler orientation of x is 90 degrees -> the bot is paralell to the x-axis

def pose_estimation(
    frame, aruco_dict_type, matrix_coefficients, distortion_coefficients
):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
        gray,
        cv2.aruco_dict,
        parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients,
    )
    
    if len(corners) > 0:
        for i in range(0, len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], 0.02, matrix_coefficients, distortion_coefficients
            )

            x_sum = (
                corners[0][0][0][0]
                + corners[0][0][1][0]
                + corners[0][0][2][0]
                + corners[0][0][3][0]
            )
            y_sum = (
                corners[0][0][0][1]
                + corners[0][0][1][1]
                + corners[0][0][2][1]
                + corners[0][0][3][1]
            )
            x_centerPixel = x_sum * 0.25
            y_centerPixel = y_sum * 0.25
            centerPixel = (int(x_centerPixel), int(y_centerPixel))
            startpoint = (int(screenWidth/2), int(screenHeight/2))
            vector_from_center(centerPixel, startpoint)
            sendThroughSerial(ids, True)

            cv2.line(frame, startpoint, centerPixel, (255, 0, 255), thickness)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            
            
            cv2.aruco.drawAxis(
                frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01
                
            )
            frame = boundingBox(frame, 120, 100, (0,0,0), 3, centerPixel, rvec, i)
        
        
    else:
        data["isoffcourse"] = False
        frame = boundingBox(frame, 120, 100, (0,0,0), 3, None, None, None)
        sendThroughSerial(ids, False)
            
    return frame

aruco_type = "DICT_ARUCO_ORIGINAL"

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])

arucoParams = cv2.aruco.DetectorParameters_create()


intrinsic_camera = np.array(
    ((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1))
)
# NOTICE The Picamera 1.3 used for this project still needs to callibrated for the above np array
# In rush so I used preset values for another camera
# Remember to callibrate camera correctly for the above and below arrays
distortion = np.array((-0.43948, 0.18514, 0, 0))


cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, screenWidth)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, screenHeight)




while cap.isOpened():
    ret, img = cap.read()

    output = pose_estimation(
        img, ARUCO_DICT[aruco_type], intrinsic_camera, distortion
    )

    cv2.imshow("Estimated Pose", output)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
