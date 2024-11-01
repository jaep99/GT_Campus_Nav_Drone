import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2
import time
import os
import platform
import sys

#############################
width=640
height=480

# Picamera2
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(
    main={"format": 'RGB888',
          "size": (width, height)}
)
picam2.configure(preview_config)
picam2.start()

viewVideo=True
if len(sys.argv)>1:
    viewVideo=sys.argv[1]
    if viewVideo=='0' or viewVideo=='False' or viewVideo=='false':
        viewVideo=False

############ARUCO/CV2############
id_to_find=72
marker_size=20 #cm
realWorldEfficiency=.7

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

calib_path="/home/alankim2003/video2calibration/calibrationFiles/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

#############################
seconds=0
if viewVideo==True:
    seconds=1000000
    print("Showing video feed if X11 enabled.")
    print("Script will run until you exit.")
    print("-------------------------------")
    print("")
else:
    seconds=5

counter=0
counter=float(counter)
start_time=time.time()

# 3D
objPoints = np.array([[-marker_size/2, marker_size/2, 0],
                     [marker_size/2, marker_size/2, 0],
                     [marker_size/2, -marker_size/2, 0],
                     [-marker_size/2, -marker_size/2, 0]], dtype=np.float32)

while time.time()-start_time<seconds:
    frame = picam2.capture_array()
    
    # BGR
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray_img)
    
    if ids is not None:
        print("Found these IDs in the frame:")
        print(ids)
        if ids[0] == id_to_find:
            # solvePnP
            success, rvec, tvec = cv2.solvePnP(objPoints, 
                                              corners[0], 
                                              cameraMatrix, 
                                              cameraDistortion)
            
            if success:
                x="{:.2f}".format(tvec[0][0])
                y="{:.2f}".format(tvec[1][0])
                z="{:.2f}".format(tvec[2][0])
                
                marker_position="MARKER POSITION: x="+x+" y="+y+" z="+z
                print(marker_position)
                print("")
                
                if viewVideo:
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.drawFrameAxes(frame, cameraMatrix, cameraDistortion, rvec, tvec, marker_size/2)
    else:
        print("ARUCO "+str(id_to_find)+" NOT FOUND IN FRAME.")
        print("")

    if viewVideo:
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    counter=float(counter+1)

if viewVideo==False:
    frequency=realWorldEfficiency*(counter/seconds)
    print("")
    print("")
    print("---------------------------")
    print("Loop iterations per second:")
    print(frequency)
    print("---------------------------")
    print("Performance Diagnosis:")
    if frequency>10:
        print("Performance is more than enough for great precision landing.")
    elif frequency>5:
        print("Performance likely still good enough for precision landing.")
        print("This resolution likely maximizes the detection altitude of the marker.")
    else:
        print("Performance likely not good enough for precision landing.")
        print("MAKE SURE YOU HAVE A HEAT SINK ON YOUR PI!!!")
    print("---------------------------")

picam2.stop()
cv2.destroyAllWindows()
