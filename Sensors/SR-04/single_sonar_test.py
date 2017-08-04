import RPi.GPIO as GPIO
import time, sys, datetime, pytz, tzlocal, os
import numpy as np
from datetime import datetime
from dateutil import tz

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pulse = 0.00002

def readSonar(direction):

    pulse_start = 0.0
    pulse_end = 0.0

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
    #print "Waiting For Sensor To Settle"
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

    # print "Duration" + pulse_duration

    distance = (pulse_duration*17092)
    distance = round(distance,2)
    return distance

def readAltitude():
    # global array_y
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

    distance = (pulse_duration*17092) - 6.92
    distance = float(round(distance,2))

    # SonarArray[array_y,5] = distance
    # array_y += 1

    return distance

while True:
    print "Distance: " + str(readAltitude())
