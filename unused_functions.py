import RPi.GPIO as GPIO                         # Needed to read ultrasonic sensors
import time, sys, datetime, pytz, tzlocal, os   # Needed for many basic functions and to satisfy preconditions
import numpy as np                              # Needed to create array
from datetime import datetime                   # Needed to generate timestamp
from dateutil import tz                         # Needed to generate timestamp
from dronekit import *                          # Needed for all drone-related commands
from pymavlink import mavutil                   # Needed for command message definitions

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pulse = 0.00002
SonarArray = np.zeros((100,6))
array_y = 0
stdata = [0,0,0,0,0,0]
np.set_printoptions(suppress=True)

def read_sonar_loop():

    #### This function reads all six SR-04 Sensors, returns results in meters, and stores all data in an array. ####
    global array_y
    for x in range (len(SonarArray[0])):        # Here we initialize the for-loop that will iterate through all 6 sensors
        pulse_start = 0.0                       # This resets variables to default state at the beginning of each iteration
        pulse_end = 0.0                         # --
        direction = x                           #

        if direction == 0:                      # This if-statement sets the Trigger and Echo GPIO pins for each sensor as we iterate
            # Front sonar
            sensortype = "Front"
            TRIG = 33
            ECHO = 37

        elif direction == 1:
            # Right sonar
            sensortype = "Right"
            TRIG = 7
            ECHO = 11

        elif direction == 2:
            # Left sonar
            sensortype = "Left"
            TRIG = 13
            ECHO = 15

        elif direction == 3:
            # Back sonar
            sensortype = "Back"
            TRIG = 16
            ECHO = 18

        elif direction == 4:
            # Top sonar
            sensortype = "Top"
            TRIG = 29
            ECHO = 31

        else:
            # Bottom sonar
            sensortype = "Bottom"
            TRIG = 32
            ECHO = 36

        GPIO.setup(ECHO,GPIO.IN)                # Sets up GPIO pins
        GPIO.setup(TRIG,GPIO.OUT)               # --

        GPIO.output(TRIG, False)
        time.sleep(0.2)

        GPIO.output(TRIG, True)
        time.sleep(pulse)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO)==0:
            pulse_start = time.time()

        while GPIO.input(ECHO)==1:
            pulse_end = time.time()
            if pulse_end - pulse_start > 0.5:
                break

        pulse_duration = pulse_end - pulse_start

        if direction == 0:
            # Front sonar
            distance = (pulse_duration*17092) - 15.2979

        elif direction == 1:
            # Right sonar
            distance = (pulse_duration*17092) - 23.7135

        elif direction == 2:
            # Left sonar
            distance = (pulse_duration*17092) - 24.2425

        elif direction == 3:
            # Back sonar
            distance = (pulse_duration*17092) - 15.7859

        elif direction == 4:
            # Top sonar
            distance = (pulse_duration*17092) - 1.098

        else:
            # Bottom sonar
            distance = (pulse_duration*17092) - 6.92

        distance = float(round(distance,2))                 # Rounds the distance to 2 decimal places
        stdata[x] = distance                                # Writes the distance to list "stdata" which will be returned
        SonarArray[array_y,x] = distance                    # Writes the distance to correct position in array "SonarArray" which will be written to the log file

    array_y += 1                                            # Increases the array y-axis by 1 to prevent overwritting the next time this function is called
    return stdata

def readAltitude():
    global array_y
    pulse_start = 0.0
    pulse_end = 0.0

    sensortype = "Bottom"
    TRIG = 32
    ECHO = 36

    GPIO.setup(ECHO,GPIO.IN)
    GPIO.setup(TRIG,GPIO.OUT)

    GPIO.output(TRIG, False)
    time.sleep(0.2)

    GPIO.output(TRIG, True)
    time.sleep(pulse)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO)==0:
        pulse_start = time.time()

    while GPIO.input(ECHO)==1:
        pulse_end = time.time()
        if pulse_end - pulse_start > 0.5:
            break

    pulse_duration = pulse_end - pulse_start

    distance = ((pulse_duration*17092) - 6.92)/100
    distance = float(round(distance,2))

    SonarArray[array_y,5] = distance
    array_y += 1

    return distance

def arm_and_takeoff(aTargetAltitude):

    #Arms vehicle and fly to aTargetAltitude.

    vehicle.parameters['ARMING_CHECK']=0
    # Don't try to arm until autopilot is ready
    # while not vehicle.is_armable:
    #     print " Waiting for vehicle to initialise..."
    #     time.sleep(1)

    print "Device initialised."
    print "Arming motors..."
    # Copter should arm in STABILIZE mode
    vehicle.mode    = VehicleMode("STABILIZE")

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        vehicle.armed   = True
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED_NOGPS")

    print vehicle.mode
    print "Motors armed."
    time.sleep(1)
    print "Taking off!"

    while True:
        sensor_data = readSonar()
        current_altitude = sensor_data[5]/100
        print(" Altitude: %s" % current_altitude)
        send_ned_velocity(0,0,-10)

        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break

    send_ned_velocity(0,0,0)
    time.sleep(2)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
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

    vehicle.send_mavlink(msg)
