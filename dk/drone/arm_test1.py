import time
import os
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

# Direct serial connection
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

def arm():
    print("Setting basic parameters...")
    # Disable arming checks
    vehicle.parameters['ARMING_CHECK']=1  # Changed to minimal checks
    
    # Set to STABILIZE mode first
    print("Setting STABILIZE mode...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(2)
    
    print("Checking vehicle status...")
    print(f"Mode: {vehicle.mode}")
    print(f"System status: {vehicle.system_status}")
    print(f"Armed: {vehicle.armed}")
    
    print("Attempting to ARM...")
    # Send direct MAVLink command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
        0,       # confirmation
        1,       # param 1 (1=arm)
        0,       # param 2 (force arm)
        0, 0, 0, 0, 0) # param 3-7 (not used)
    
    # Send command
    vehicle.send_mavlink(msg)
    
    # Wait for arming
    timeout = 10  # 10 seconds timeout
    start = time.time()
    while not vehicle.armed:
        if time.time() - start > timeout:
            print("ARM timeout!")
            break
        print("Waiting for ARM...")
        time.sleep(1)
    
    if vehicle.armed:
        print("ARM complete!")
    else:
        print("Failed to ARM!")
        print(f"Final mode: {vehicle.mode}")
        print(f"Final status: {vehicle.system_status}")
    
    return vehicle.armed

if __name__=='__main__':
    try:
        print("Connecting to drone...")
        print(f"Initial mode: {vehicle.mode}")
        print(f"Initial system status: {vehicle.system_status}")
        
        success = arm()
        
        if success:
            print("Successfully armed! Waiting 5 seconds...")
            time.sleep(5)
            print("Disarming...")
            # Disarm
            msg = vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0)
            vehicle.send_mavlink(msg)
            
        vehicle.close()
        
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        if 'vehicle' in locals():
            vehicle.close()
