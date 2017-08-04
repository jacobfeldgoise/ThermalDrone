import RPi.GPIO as GPIO
import time, sys, datetime, pytz, tzlocal, os, pickle, multiprocessing
import numpy as np
from datetime import datetime
from dateutil import tz


GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pulse = 0.00002
SonarArray = np.zeros((50,6))
array_y = 0
stdata = [0,0,0,0,0,0]
np.set_printoptions(suppress=True)

sensor_loop = multiprocessing.Event()

def read_individual_sonar(direction, stdata):

    #### This function reads one SR-04 Sensor, returns results in meters, and stores all data in an array. ####
    #### The argument "direction" must be an integer from 0 to 5, inclusive ####

    pulse_start = 0.0                       # This resets variables to default state at the beginning of each iteration
    pulse_end = 0.0                         # --

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

    elif direction == 5:
        # Bottom sonar
        sensortype = "Bottom"
        TRIG = 32
        ECHO = 36
    else:
        raise ValueError("The argument 'direction' must be an integer from 0 to 5, inclusive!")

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

    elif direction == 5:
        # Bottom sonar
        distance = (pulse_duration*17092) - 6.92

    distance = float(round(distance,2))                         # Rounds the distance to 2 decimal places
    SonarArray[array_y,direction] = distance                    # Writes the distance to correct position in array "SonarArray" which will be written to the log file
    stdata[direction] = distance                                # Writes the distance to global list "stdata"


def read_sonar_multi_call(sensor_loop, stdata_q):

    while sensor_loop.is_set() == False:

        pool = multiprocessing.Pool(processes=6)
        stdata_pre = [pool.apply_async(read_individual_sonar, args=()) for x in range(6)]
        stdata = [p.get() for p in stdata_pre]
        print "Stdata in sub-process: " + str(stdata)
        stdata_q.put(stdata)                                    # Sends stdata back to main process

        for x in stdata:                                        # Writes stdata to the correct row of array "SonarArray"
            SonarArray[array_y,x] = stdata[x]                   # --
        array_y += 1                                            # Increases the array y-axis by 1 to prevent overwritting the next time this function is called

    # Logs SonarArray to seperate file and prints it to the console
    # print "Logging data..."
    # logdata()
    # print "Data logged!"
    print "Printing SonarArray..."
    print SonarArray

if __name__ == "__main__":
    # All code -- including calling pre-defined functions -- goes here

    stdata_q = multiprocessing.Queue()
    proc = multiprocessing.Process(target=read_sonar_multi_call, args=(sensor_loop, stdata_q))
    proc.start()

    for number in range(5):
        print stdata
        time.sleep(0.5)

    sensor_loop.set()
    proc.join()

GPIO.cleanup()
