import RPi.GPIO as GPIO
from time import *
import numpy as np
import sys

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def readSonar(direction):

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


    # elif direction == "4":
    #     # Top sonar
    #     TRIG = 33
    #     ECHO = 37
    #
    # else:
    #     # Bottom sonar
    #     TRIG = 32
    #     ECHO = 36

    maxSonarRange = 400
    pulse_start = 0.0
    pulse_end = 0.0
    initTime = time()

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)

    GPIO.output(TRIG, False)
    #print "Waiting For Sensor To Settle"
    sleep(0.5)

    GPIO.output(TRIG, True)
    sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO)==0:
        pulse_start = time()

    while GPIO.input(ECHO)==1:
        pulse_end = time()
        if time() - initTime > 0.5:
            break
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    # if distance > maxSonarRange:
    #     distance = maxSonarRange

    return distance

SonarArray = np.zeros((4,4))

# for y in range (len(SonarArray)):
#     for x in range (len(SonarArray[0])):
#         SonarArray[y,x] = readSonar(x)
# print SonarArray

print readSonar(0)

GPIO.cleanup()
