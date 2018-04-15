#!/usr/bin/env python
import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(0)
markerLength = 350
camera_matrix = np.asmatrix([[613.299988, 0.0, 354.949005],[0.0, 0.0, 612.106018],[214.380005, 0.0, 1.0]])
dist_coeffs = (-0.439, 0.263, 0.001, 0.0, -0.113)
 
while(True):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if type(ids) != type(None):
        rvec,tvec,z = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        #rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        imgWithAruco = aruco.drawDetectedMarkers(gray, corners, ids)
        imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 100)
        cv2.imshow('frame',imgWithAruco)
    else:
        pass
    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
