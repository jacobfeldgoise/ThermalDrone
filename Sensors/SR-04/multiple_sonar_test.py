import RPi.GPIO as GPIO
import time, sys, datetime, pytz, tzlocal, os
import numpy as np
from datetime import datetime
from dateutil import tz

np.set_printoptions(suppress=True)

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
    return [distance,sensortype]

# while True:
#     for x in range (0,6):
#         print readSonar(x)[1] + " Distance:" + str(readSonar(x)[0])

SonarArray = np.empty((20,6))
avg_list = []

for y in range (len(SonarArray)):
    for x in range (len(SonarArray[0])):
        sensorData = readSonar(x)
        SonarArray[y,x] = sensorData[0]
        #
        # if sensorData[0]>500:
        #     SonarArray[y,x] = 0
        #
        # else:
        #     # print sensorData[0]
    avg_list.append(SonarArray[y,5])

def timestamp():
    local_timezone = tzlocal.get_localzone()
    utc_time = datetime.utcnow()

    # Convert time zone
    local_time= utc_time.replace(tzinfo=pytz.utc).astimezone(local_timezone)
    timestamp_local = local_time.strftime('%m-%d-%Y %H:%M:%S %Z')
    return timestamp_local

print "Writing to Log..."
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

print sum(avg_list) / float(len(avg_list))

logfile.close()
GPIO.cleanup()
