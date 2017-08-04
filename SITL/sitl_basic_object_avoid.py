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
np.set_printoptions(suppress=True)
SonarArray = np.zeros((100,6))
array_y = 0
stdata = [0,0,0,0,0,0]

target = sys.argv[1] if len(sys.argv) >= 2 else 'tcp:127.0.0.1:5760'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

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

def arm_and_takeoff_stabilize(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    # vehicle.parameters['ARMING_CHECK']=0

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.3
    SMOOTH_TAKEOFF_THRUST = 0.2

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in STABILIZE mode
    vehicle.mode = VehicleMode("STABILIZE")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)


    print(" Vehicle Armed!")
    # Switch to Copter to GUIDED_NOGPS Mode for Takeoff
    # vehicle.mode = VehicleMode("GUIDED_NOGPS")

    print("Taking off!")
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        sensor_data = readSonar()
        current_altitude = sensor_data[5]/100

        print(" Altitude: %s" % current_altitude)

        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break

        elif current_altitude >= aTargetAltitude*0.6: #Slighlty decrease thrust at 0.6 x Target Altitude
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)

def readSonar():
    global array_y
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
    return stdata

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
    print "Switching to STABILIZE..."
    vehicle.mode    = VehicleMode("STABILIZE")

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

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """

    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """

    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
                                                             0,
                                                             0,
                                                                 # Target system
                                                             0,
                                                                 # Target component
                                                             0b00000000,
                                                                 # Type mask: bit 1 is LSB
                                                             to_quaternion(roll_angle, pitch_angle),
                                                                 # Quaternion
                                                             0,
                                                                 # Body roll rate in radian
                                                             0,
                                                                 # Body pitch rate in radian
                                                             math.radians(yaw_rate),
                                                                 # Body yaw rate in radian
                                                             thrust)
                                                                 # Thrust
    vehicle.send_mavlink(msg)

    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)

        # Sleep for the fractional part
        time.sleep(modf[0])

        # Send command to vehicle on 1 Hz cycle
        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

arm_and_takeoff_stabilize(1)

t = 0

while True:
    sensor_data = readSonar()

    if sensor_data[1] < 20:
        print "Leaning Left..."
        set_attitude(0.5,-0.1,0)

    elif sensor_data[2] < 20:
        print "Leaning Right..."
        send_ned_velocity(0.5,0.1,0)

    elif sensor_data[1] > 30 and sensor_data[1] < sensor_data[2]:
        print "Leaning Right..."
        send_ned_velocity(0.5,0.1,0)

    elif sensor_data[2] > 30 and sensor_data[2] < sensor_data[1]:
        print "Leaning Left..."
        send_ned_velocity(0.5,-0.1,0)

    elif sensor_data[1] > 50 and sensor_data[2] > 50:
        break

    else:
        send_ned_velocity(0.5,0,0)
    t += 1

    if t == 4:
        break


time.sleep(1)
land_and_close()
logdata()
GPIO.cleanup()
