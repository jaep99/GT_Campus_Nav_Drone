import time
import os
import argparse
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

# Configuration
TIMEOUT = 30

def get_arguments():
    parser = argparse.ArgumentParser(description='Drone takeoff and landing script')
    parser.add_argument('--altitude', type=float, default=1.0,
                      help='Target altitude in meters (default: 1.0)')
    parser.add_argument('--port', type=str, default='/dev/ttyAMA0',
                      help='Port for vehicle connection (default: /dev/ttyAMA0)')
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

def land():
    print("Switching to LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    
    start = time.time()
    while vehicle.mode != 'LAND':
        if time.time() - start > TIMEOUT:
            print("Mode change timeout!")
            break
        print("Waiting for LAND mode...")
        time.sleep(1)
    
    while vehicle.armed:
        print(f"Current Altitude: {vehicle.location.global_relative_frame.alt:.1f}m")
        time.sleep(1)
    
    print("Landing complete!")

if __name__=='__main__':
    try:
        # Parse arguments
        args = get_arguments()
        
        print(f"Connecting to vehicle on {args.port}...")
        vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
        
        print("Starting mission...")
        print(f"Target altitude: {args.altitude}m")
        print(f"Initial mode: {vehicle.mode}")
        print(f"Initial system status: {vehicle.system_status}")
        
        # Arm the vehicle
        if arm_vehicle():
            # Takeoff
            takeoff(args.altitude)
            
            # Hover for a moment
            print("Hovering...")
            time.sleep(1)
            
            # Land
            land()
            
        vehicle.close()
        print("Mission complete!")
        
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        if 'vehicle' in locals():
            vehicle.close()
