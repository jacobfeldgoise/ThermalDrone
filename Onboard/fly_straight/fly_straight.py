# Connect to the Vehicle (in this case a simulator running the same computer)
from dronekit import *
import sys
import time

# Connect to UDP endpoint (and wait for default attributes to accumulate)
target = sys.argv[1] if len(sys.argv) >= 2 else 'udpin:0.0.0.0:14550'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):

    #Arms vehicle and fly to aTargetAltitude.

    print "Basic pre-arm checks..."
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Device initialised."
    print "Arming motors..."
    # Copter should arm in STABILIZE mode
    vehicle.mode    = VehicleMode("STABILIZE")

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        vehicle.armed   = True
        time.sleep(1)

    print "Motors armed."
    time.sleep(1)
    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude)
    time.sleep(5)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    #Move vehicle in direction based on specified velocity vectors.

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def land_and_close():
    print "Switching to LOITER..."
    vehicle.mode    = VehicleMode("LOITER")

    print "Stabilizing..."
    time.sleep(2)

    print "Landing..."
    vehicle.mode = VehicleMode("LAND")

    print "Closing vehicle object..."
    vehicle.close()
    time.sleep(1)
    print "Vehicle Disarmed."

arm_and_takeoff(2)

print "Switching to LOITER..."
vehicle.mode    = VehicleMode("LOITER")

print "Flying..."
send_ned_velocity(2, 0, 0, 3)

print "Stabilizing..."
time.sleep(2)

print "Flying..."
send_ned_velocity(-2, 0, 0, 3)

land_and_close()
