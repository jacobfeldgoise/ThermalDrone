# Connect to the Vehicle (in this case a simulator running the same computer)
from dronekit import *
import sys
import time

# Connect to UDP endpoint (and wait for default attributes to accumulate)
target = sys.argv[1] if len(sys.argv) >= 2 else 'udpin:0.0.0.0:14550'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks..."
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Device initialised."
    print "Arming motors..."
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Motors armed."
    time.sleep(1)
    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    # while True:
    #     print " Altitude: ", vehicle.location.global_relative_frame.alt
    #     #Break and return from function just below target altitude.
    #     if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
    #         print "Reached target altitude"
    #         break
    time.sleep(4)

arm_and_takeoff(2)

print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print "Closing vehicle object..."
vehicle.close()
time.sleep(1)
print "Vehicle Disarmed."
