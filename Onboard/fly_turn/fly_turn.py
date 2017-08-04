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

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

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

arm_and_takeoff(3)
time.sleep(2)

print "Turning..."
condition_yaw(90, True)
time.sleep(4)
