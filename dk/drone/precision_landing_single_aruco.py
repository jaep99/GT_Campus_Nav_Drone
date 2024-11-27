###########DEPENDENCIES################
import time
import socket
import exceptions
import math
import argparse
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2

# Configuration
TIMEOUT = 30

#######VARIABLES####################
# Camera Setup
width = 640
height = 480
viewVideo = True  # Can be overridden by command line argument

# Aruco Setup
id_to_find = 72
marker_size = 20  # cm
takeoff_height = 5
velocity = 0.5

# Initialize Camera
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(
    main={"format": 'RGB888',
          "size": (width, height)}
)
picam2.configure(preview_config)
picam2.start()

# Aruco Detection Setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Camera Calibration
calib_path = "/home/alankim2003/video2calibration/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

# 3D points for solvePnP
objPoints = np.array([[-marker_size/2, marker_size/2, 0],
                     [marker_size/2, marker_size/2, 0],
                     [marker_size/2, -marker_size/2, 0],
                     [-marker_size/2, -marker_size/2, 0]], dtype=np.float32)

##Counters and script triggers
found_count = 0
notfound_count = 0
first_run = 0
start_time = 0
end_time = 0

def get_arguments():
    parser = argparse.ArgumentParser(description='Drone precision landing script')
    parser.add_argument('--altitude', type=float, default=takeoff_height,
                      help=f'Target altitude in meters (default: {takeoff_height})')
    parser.add_argument('--port', type=str, default='/dev/ttyAMA0',
                      help='Port for vehicle connection (default: /dev/ttyAMA0)')
    parser.add_argument('--novideo', action='store_true',
                      help='Disable video display')
    return parser.parse_args()

def arm_vehicle():
    print("Setting basic parameters...")
    vehicle.parameters['ARMING_CHECK'] = 1
    
    print("Setting STABILIZE mode...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(2)
    
    print("Checking vehicle status...")
    print(f"Mode: {vehicle.mode}")
    print(f"System status: {vehicle.system_status}")
    
    print("Attempting to ARM...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,       
        1,       
        0,       
        0, 0, 0, 0, 0)
    
    vehicle.send_mavlink(msg)
    
    start = time.time()
    while not vehicle.armed:
        if time.time() - start > TIMEOUT:
            print("ARM timeout!")
            return False
        print("Waiting for ARM...")
        time.sleep(1)
    
    print("ARM complete!")
    return True

def takeoff(target_altitude):
    print("Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        print("Waiting for GUIDED mode...")
        time.sleep(1)
    
    print(f"Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)
    
    start = time.time()
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Current Altitude: {current_altitude:.1f}m")
        
        if current_altitude >= target_altitude * 0.95:
            print("Target altitude reached!")
            break
            
        if time.time() - start > TIMEOUT:
            print("Takeoff timeout!")
            break
            
        time.sleep(1)

def detect_and_process_marker(frame):
    # Convert from RGB to BGR
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray_img)
    
    marker_found = False
    x_ang = 0
    y_ang = 0
    
    if ids is not None and ids[0] == id_to_find:
        marker_found = True
        # Get pose
        success, rvec, tvec = cv2.solvePnP(objPoints, 
                                          corners[0], 
                                          cameraMatrix, 
                                          cameraDistortion)
        
        if success:
            x = '{:.2f}'.format(tvec[0][0])
            y = '{:.2f}'.format(tvec[1][0])
            z = '{:.2f}'.format(tvec[2][0])
            
            print(f"MARKER POSITION: x={x} y={y} z={z}")
            
            # Draw marker and axes if video display is enabled
            if viewVideo:
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, cameraMatrix, cameraDistortion, rvec, tvec, marker_size/2)
            
            # Calculate angles for precision landing
            x_sum = sum(corner[0] for corner in corners[0][0])
            y_sum = sum(corner[1] for corner in corners[0][0])
            
            x_avg = x_sum * 0.25
            y_avg = y_sum * 0.25
            
            x_ang = (x_avg - width * 0.5) * (horizontal_fov/width)
            y_ang = (y_avg - height * 0.5) * (vertical_fov/height)
    
    return frame, marker_found, x_ang, y_ang

def lander():
    global first_run, notfound_count, found_count, start_time
    
    if first_run == 0:
        print("First run of lander!!")
        first_run = 1
        start_time = time.time()

    # GPS Informations
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt
    velocity = vehicle.velocity
    
    print(f"GPS: pos=({lat:.6f}, {lon:.6f}), alt={alt:.1f}m, vel=({velocity[0]:.1f}, {velocity[1]:.1f}, {velocity[2]:.1f})m/s")

    frame = picam2.capture_array()
    frame, marker_found, x_ang, y_ang = detect_and_process_marker(frame)
    
    if vehicle.mode != 'LAND':
        vehicle.mode = VehicleMode("LAND")
        while vehicle.mode != 'LAND':
            print('WAITING FOR DRONE TO ENTER LAND MODE')
            time.sleep(1)
    
    if marker_found:
        send_land_message(x_ang, y_ang)
        found_count += 1
    else:
        notfound_count += 1
        print("ARUCO MARKER NOT FOUND")
    
    if viewVideo:
        cv2.imshow('Precision Landing Camera Feed', frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        # Parse arguments
        args = get_arguments()
        
        print(f"Connecting to vehicle on {args.port}...")
        vehicle = connect(args.port, wait_ready=True, baud=57600)
        
        # Set precision landing parameters
        vehicle.parameters['PLND_ENABLED'] = 1
        vehicle.parameters['PLND_TYPE'] = 1
        vehicle.parameters['PLND_EST_TYPE'] = 0
        vehicle.parameters['LAND_SPEED'] = 20
        
        print("Starting mission...")
        print(f"Target altitude: {args.altitude}m")
        print(f"Initial mode: {vehicle.mode}")
        print(f"Initial system status: {vehicle.system_status}")
        
        # Arm and takeoff
        if arm_vehicle():
            takeoff(args.altitude)
            
            # Hover for a moment
            print("Hovering...")
            time.sleep(1)
            
            # Start precision landing sequence
            ready_to_land = 1
            if ready_to_land == 1:
                while vehicle.armed:
                    lander()
                end_time = time.time()
                total_time = end_time - start_time
                total_time = abs(int(total_time))

                total_count = found_count + notfound_count
                freq_lander = total_count/total_time
                print("Total iterations: " + str(total_count))
                print("Total seconds: " + str(total_time))
                print("------------------")
                print("lander function had frequency of: " + str(freq_lander))
                print("------------------")
                print("Vehicle has landed")
                print("------------------")
            
        # Cleanup
        vehicle.close()
        picam2.stop()
        cv2.destroyAllWindows()
        print("Mission complete!")
        
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        if 'vehicle' in locals():
            vehicle.close()
        if 'picam2' in locals():
            picam2.stop()
        cv2.destroyAllWindows()
