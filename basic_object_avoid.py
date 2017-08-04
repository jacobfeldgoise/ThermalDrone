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

target = sys.argv[1] if len(sys.argv) >= 2 else 'udpin:0.0.0.0:14550'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

def arm_and_takeoff_stabilize(aTargetAltitude):

    #### This function arms vehicle and flies to aTargetAltitude without GPS data in STABILIZE Mode. ####

    # The following command disables all arming checks. Be careful, this is very dangerous: vehicle.parameters['ARMING_CHECK']=0

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.4
    SMOOTH_TAKEOFF_THRUST = 0.1

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors...")
    vehicle.mode = VehicleMode("STABILIZE") # Copter should arm in STABILIZE mode
    print "Mode: " + vehicle.mode.name

    vehicle.armed = True                    # Arms Vehicle
    vehicle.flush()                         # Forces DroneKit to send all outstanding messages

    while not vehicle.armed:                # This loops ensures that the vehicle is armed before attempting takeoff
        print(" Waiting for arming...")
        time.sleep(1)

    print(" Vehicle Armed!")

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    set_attitude_alt(thrust = 0.6)

    while True:
        current_altitude = readAltitude()
        print " Altitude: " + str(current_altitude)

        if current_altitude > 0.8: # Trigger just below target alt.
            print("Reached target altitude")
            break

    # while True:
    #     current_altitude = readAltitude()/100
    #     print " Altitude: " + str(current_altitude)
    #
    #     if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
    #         print("Reached target altitude")
    #         break
    #
    #     elif current_altitude >= aTargetAltitude*0.7: #Slighlty decrease thrust at 0.6 x Target Altitude
    #         thrust = SMOOTH_TAKEOFF_THRUST
    #         print("Smoothing thrust...")
    #
    #     set_attitude(thrust = thrust)

def timestamp():

    #### This function generates a timestemp which will be written to the log file ####

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
    print "Switching to STABILIZE mode to begin landing..."
    vehicle.mode    = VehicleMode("STABILIZE")

    print "Stabilizing & Ceasing all movement..."
    send_ned_velocity(0.5,0.1,0)


    time.sleep(2)

    print "Landing..."
    vehicle.mode = VehicleMode("LAND")

    print "Closing vehicle object..."
    vehicle.close()
    time.sleep(1)
    print "Vehicle Disarmed."

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

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.005, duration = 0):
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
    # set_roll_pitch_yaw_thrust_encode
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

def set_attitude_alt(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """


    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend

    msg = vehicle.message_factory.set_roll_pitch_yaw_thrust_encode(
                                                             0,                         # Target system
                                                             0,                         # Target component
                                                             math.radians(roll_angle),  # Body roll rate in radian
                                                             math.radians(pitch_angle), # Body pitch rate in radian
                                                             math.radians(yaw_angle),   # Body yaw angle in radian
                                                             thrust)                    # Thrust
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

# while True:
#     sensor_data = readSonar()
#
#     if sensor_data[1] < 20:
#         print "Leaning Left..."
#         set_attitude(0.5,-0.1,0)
#
#     elif sensor_data[2] < 20:
#         print "Leaning Right..."
#         send_ned_velocity(0.5,0.1,0)
#
#     elif sensor_data[1] > 30 and sensor_data[1] < sensor_data[2]:
#         print "Leaning Right..."
#         send_ned_velocity(0.5,0.1,0)
#
#     elif sensor_data[2] > 30 and sensor_data[2] < sensor_data[1]:
#         print "Leaning Left..."
#         send_ned_velocity(0.5,-0.1,0)
#
#     elif sensor_data[1] > 50 and sensor_data[2] > 50:
#         break
#
#     else:
#         send_ned_velocity(0.5,0,0)
#     t += 1
#
#     if t == 4:
#         break


time.sleep(1)
land_and_close()
print "Logging data..."
logdata()
print "Data logged!"
GPIO.cleanup()
