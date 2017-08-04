import RPi.GPIO as GPIO
import time, sys, datetime, pytz, tzlocal, os
import numpy as np
from datetime import datetime
from dateutil import tz
from dronekit import *

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

def readSonar():
    for x in range (len(SonarArray[0])):
        pulse_start = 0.0
        pulse_end = 0.0
        direction = x

        if direction == 0:
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

        distance = float(round(distance,2))
        stdata[x] = distance
        SonarArray[array_y,x] = distance

    array_y += 1

def timestamp():
    local_timezone = tzlocal.get_localzone()
    utc_time = datetime.utcnow()

    # Convert time zone
    local_time= utc_time.replace(tzinfo=pytz.utc).astimezone(local_timezone)
    timestamp_local = local_time.strftime('%m-%d-%Y %H:%M:%S %Z')
    return timestamp_local

def logdata():
    if os.path.isfile("/home/pi/sensorlog.txt"):
        if os.stat("/home/pi/sensorlog.txt").st_size == 0:
            logfile = open("sensorlog.txt","w")
            logfile.write(timestamp())
            logfile.write("\n" + np.array_str(SonarArray))

        else:
            logfile = open("sensorlog.txt","a")
            logfile.write("\n" + "\n" + timestamp())
            logfile.write("\n" + np.array_str(SonarArray))
    else:
        logfile = open("sensorlog.txt","w")
        logfile.write(timestamp())
        logfile.write("\n" + np.array_str(SonarArray))

    logfile.close()

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

def condition_yaw(heading, direction):
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,  # param 3, direction -1 ccw, 1 cw
        1,          # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

target = sys.argv[1] if len(sys.argv) >= 2 else 'udpin:0.0.0.0:14550'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pulse = 0.00002
np.set_printoptions(suppress=True)
SonarArray = np.zeros((100,6))
array_y = 0
stdata = [0,0,0,0,0,0]

arm_and_takeoff(1)

while True:

    readSonar()
    if readSonar[0] < 30 and readSonar[1] > 30:
        send_ned_velocity(0,0,0)
        print "Turning..."
        condition_yaw(180,-1)

    elif readSonar[0] > 30 and readSonar[1] > 30:
        send_ned_velocity(0,0,0)
        print "Turning..."
        condition_yaw(90,1)

    elif readSonar[0] < 30 and readSonar[1] < 30:
        send_ned_velocity(0,0,0)
        print "Turning..."
        condition_yaw(90,-1)

    else:
        if readSonar[1] < 20:
            print "Leaning Left..."
            send_ned_velocity(0.5,-0.25,0)

        elif readSonar[2] < 20:
            print "Leaning Right..."
            send_ned_velocity(0.5,0.25,0)
        elif


land_and_close()
logdata()
GPIO.cleanup()
