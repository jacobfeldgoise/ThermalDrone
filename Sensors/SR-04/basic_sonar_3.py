
import RPi.GPIO as GPIO
import time, sys, datetime, pytz, tzlocal, os
import numpy as np
from datetime import datetime
from dateutil import tz

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pulse = 0.00002

# gpio_inputs = [18,11,15,31]
# gpio_outputs = [16,7,13,31]
#
# for x in gpio_inputs:
#     GPIO.setup(x,GPIO.IN)
#     print "Setting up input: " + str(x)
#
# for x in gpio_outputs:
#     GPIO.setup(x,GPIO.OUT)
#     print "Setting up output: " + str(x)

def readSonar(direction):

    pulse_start = 0.0
    pulse_end = 0.0

    if direction == 0:
        # Front sonar
        sensortype = "Front"
        TRIG = 16
        ECHO = 18

    elif direction == 1:
        # Left sonar
        sensortype = "Left"
        TRIG = 7
        ECHO = 11

    elif direction == 2:
        # Right sonar
        sensortype = "Right"
        TRIG = 13
        ECHO = 15

    else:
        # Back sonar
        sensortype = "Back"
        TRIG = 29
        ECHO = 31

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

    distance = (pulse_duration*17092) + 0.352
    distance = round(distance,2)
    return [distance,sensortype]

# sensorData = readSonar(0)
# print sensorData
# print "Distance (" + sensorData[1] + " sensor): " + str(sensorData[0]) + "cm"

SonarArray = np.empty((20,4))

for y in range (len(SonarArray)):
    for x in range (len(SonarArray[0])):
        sensorData = readSonar(x)
        # print sensorData[0]
        SonarArray[y,x] = sensorData[0]

def timestamp():
    local_timezone = tzlocal.get_localzone()
    utc_time = datetime.utcnow()

    # Convert time zone
    local_time= utc_time.replace(tzinfo=pytz.utc).astimezone(local_timezone)
    timestamp_local = local_time.strftime('%m-%d-%Y %H:%M:%S %Z')
    return timestamp_local


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


print SonarArray

avg_list = []
for x in range (len(SonarArray)):
    avg_list.append(SonarArray[x][3])

print sum(avg_list) / float(len(avg_list))

logfile.close()
GPIO.cleanup()
