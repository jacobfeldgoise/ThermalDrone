import RPi.GPIO as GPIO
import time, sys, datetime, pytz, tzlocal, os, pickle, multiprocessing
from datetime import datetime
from dateutil import tz

def timestamp():

    #### This function generates a timestemp which will be written to the log file ####

    local_timezone = tzlocal.get_localzone()
    utc_time = datetime.utcnow()

    # Convert time zone
    local_time= utc_time.replace(tzinfo=pytz.utc).astimezone(local_timezone)
    timestamp_local = local_time.strftime('%m-%d-%Y %H:%M:%S %Z')
    return timestamp_local

def read_individual_sonar(direction):

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
    return distance

def logdata(list_to_write, logfile):
    #### This function writes a string to the sensor log file ####

    print "[Sub-process]: Logging data..."
    logfile.write("\n" + "Sonar Data: " + list_to_write)
    print "[Sub-process]: Data logged!"

def read_sonar_multi_call(sensor_loop, stdata_q):

    print "[Sub-process]: Opening sensor log file and timestamping..."
    if os.path.isfile("/home/pi/sensorlog.txt") and os.stat("/home/pi/sensorlog.txt").st_size != 0:         # Here we initialize the sensor log file and write the timestamp
        logfile = open("sensorlog.txt","a")
        logfile.write("\n" + "\n" + timestamp())
    else:
        logfile = open("sensorlog.txt","w")
        logfile.write(timestamp())

    print "[Sub-process]: Initializing pool to read sonar data..."
    while sensor_loop.is_set() == False:

        pool = multiprocessing.Pool(processes=6)
        stdata_pre = [pool.apply_async(read_individual_sonar, args=(x,)) for x in range(6)]
        stdata = [p.get() for p in stdata_pre]

        logdata(str(stdata), logfile)
        stdata_q.put(stdata)                                                # Sends "stdata" back to main process via multiprocessing queue

    print "[Sub-process]: Closing sensor log file..."
    logfile.close()
    print "[Sub-process]: Rejoining master process..."

if __name__ == "__main__":
    # All code -- including calling pre-defined functions -- goes here

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    pulse = 0.00002
    stdata = [0,0,0,0,0,0]
    sensor_loop = multiprocessing.Event()

    print "Initializing queue to communicate between processes..."
    stdata_q = multiprocessing.Queue()
    print "Queue initialised!"
    proc = multiprocessing.Process(target=read_sonar_multi_call, args=(sensor_loop, stdata_q,))
    print "Starting sub-process to control sensor reading and writing..."
    proc.start()
    print "Process started!"

    time.sleep(0.5)
    for number in range(10):
        print "Getting list of sonar distances from sub-process..."
        stdata = stdata_q.get()
        print stdata
        time.sleep(0.1)

    print "Triggering sensor_loop event to end sub-process..."
    sensor_loop.set()
    print "Cleaning up sub-process..."
    proc.join()
    print "Done!"

GPIO.cleanup()
