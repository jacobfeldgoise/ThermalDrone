import time, sys, datetime, pytz, tzlocal, os, pickle, multiprocessing          # Needed for many basic functions and to satisfy preconditions
from datetime import datetime                                                   # Needed to generate timestamp
from dateutil import tz                                                         # Needed to generate timestamp
from dronekit import *                                                          # Needed for all drone-related commands
from pymavlink import mavutil

def set_attitude_alt(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = 0.0, thrust = 0.2, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """


    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend

    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT, #command
        0, #confirmation
        0,          # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 1)    # param 5 ~ 7 not used
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

    # Note that from AC3.3 the message should be re-sent every second (after about 3 seconds with no message the velocity will drop back to zero).
    # In AC3.2.1 and earlier the specified velocity persists until it is canceled.
    # The code below should work on either version (sending the message multiple times does not cause problems).

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

def arm_and_takeoff_stabilize(aTargetAltitude):

    #### This function arms vehicle and flies to aTargetAltitude without GPS data in STABILIZE Mode. ####

    # The following command disables all arming checks. Be careful, this is very dangerous: vehicle.parameters['ARMING_CHECK']=0


    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.2
    SMOOTH_TAKEOFF_THRUST = 0.2

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.

    # while not vehicle.is_armable:
    #     print(" Waiting for vehicle to initialise...")
    #     time.sleep(1)

    while vehicle.mode != VehicleMode("ALT_HOLD"):
        print("Waiting for ALT_HOLD...")
        vehicle.mode = VehicleMode("ALT_HOLD") # Copter should arm in STABILIZE mode
        print "Mode: " + vehicle.mode.name
        time.sleep(0.2)

    time.sleep(1)

    vehicle.armed = True                    # Arms Vehicle
    vehicle.flush()                         # Forces DroneKit to send all outstanding messages

    print("Arming motors...")
    while not vehicle.armed:                # This loops ensures that the vehicle is armed before attempting takeoff
        print(" Waiting for arming...")
        time.sleep(1)

    print("Vehicle Armed!")

    print("Confirming Mode...")
    while vehicle.mode != "ALT_HOLD":
        vehicle.mode = VehicleMode("ALT_HOLD") # Copter should arm in STABILIZE mode
        print "Mode: " + vehicle.mode.name

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST

    set_attitude()
    vehicle.flush()                         # Forces DroneKit to send all outstanding messages

target = sys.argv[1] if len(sys.argv) >= 2 else 'tcp:127.0.0.1:5760'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

arm_and_takeoff_stabilize(1)
