import RPi.GPIO as GPIO
from time import *
import time
import numpy as np
import sys

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

def get_distance (direction):

    if direction == "0":
        # Front sonar
        TRIG = 16
        ECHO = 18

    elif direction == "1":
        # Left sonar
        TRIG = 7
        ECHO = 11

    elif direction == "2":
        # Right sonar
        TRIG = 13
        ECHO = 15

    else:
        # Back sonar
        TRIG = 29
        ECHO = 31

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    GPIO.output (TRIG,False)                                             # Initially turn them all off

    if GPIO.input (ECHO):                                               # If the 'ECHO' pin is already high
        return (100)                                                    # then exit with 100 (sensor fault)

    distance = 0                                                        # Set initial distance to zero

    GPIO.output (TRIG,False)                                            # Ensure the 'TRIG' pin is low for at
    time.sleep (0.1)                                                   # least 50mS (recommended re-sample time)

    GPIO.output (TRIG,True)                                             # Turn on the 'TRIG' pin for 10uS (ish!)
    time.sleep(0.0001)

    GPIO.output (TRIG,False)                                            # Turn off the 'TRIG' pin
    time1 = time.time(), time.time()                             # Set inital time values to current time

    # while not GPIO.input (ECHO):                                        # Wait for the start of the 'ECHO' pulse
    #     time1 = time.time()                                             # Get the time the 'ECHO' pin goes high
    #     if time1 - time2 > 0.02:                                        # If the 'ECHO' pin doesn't go high after 20mS
    #         distance = 101                                              # then set 'distance' to 100
    #         break                                                       # and break out of the loop
    #
    # if distance == 101:                                                 # If a sensor error has occurred
    #     return (distance)                                               # then exit with 100 (sensor fault)

    while GPIO.input (ECHO)==1:                                            # Otherwise, wait for the 'ECHO' pin to go low
        time2 = time.time()                                             # Get the time the 'ECHO' pin goes low
        #if time2 - time1 > 0.02:                                        # If the 'ECHO' pin doesn't go low after 20mS
        #    distance = 102                                              # then ignore it and set 'distance' to 100
        #    break                                                       # and break out of the loop

    #if distance == 102:                                                 # If a sensor error has occurred
    #    return (distance)                                               # then exit with 100 (sensor fault)

                                                                        # Sound travels at approximately 2.95uS per mm
                                                                        # and the reflected sound has travelled twice                                                        # the distance we need to measure (sound out,
                                                                        # bounced off object, sound returned)

    distance = (time2 - time1) / 0.00000295 / 2 / 10                    # Convert the timer values into centimetres
    return (distance)                                                   # Exit with the distance in centimetres

print get_distance(0)

GPIO.cleanup()
