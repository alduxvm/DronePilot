#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pixVehicle.py -> Module that contains several common functions for pixhawk vehicles. Updated to DroneKit 2.0 """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Aldux.net"

__license__ = "GPL"
__version__ = "2.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time
from math import sqrt, pow
from dronekit import connect, VehicleMode
from pymavlink import mavutil

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    if vehicle.armed and vehicle.location.global_relative_frame.alt > 2:
        print "\n\tVehicle armed and possible flying, aborting take off!\n"
        return
    print "Basic pre-arm checks"
    # Don't let the user try to fly autopilot is booting
    if vehicle.mode.name == "INITIALISING":
        print "Waiting for vehicle to initialise"
        time.sleep(1)
    while vehicle.gps_0.fix_type < 2:
        print "Waiting for GPS...:", vehicle.gps_0.fix_type
        time.sleep(1)
        
    print "Arming motors"
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    while not vehicle.armed:
        print "Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    try:
        while vehicle.mode.name=="GUIDED":
            print " -> Alt:", vehicle.location.global_relative_frame.alt
            if abs(vehicle.location.global_relative_frame.alt-aTargetAltitude) < 0.05: 
                print "\n\tReached %0.1f m\n" % (aTargetAltitude)
                break
            time.sleep(1)
    except KeyboardInterrupt:
        print "Keyboard Interrupt on arm_and_takeoff."
        pass # do cleanup here


def go_to_alt(vehicle, target):
    """
    Function that makes the vehicle travel to an specific lat/lon location. Measures distance and if the target is reached.
    """
    timeout = 20
    condition_yaw(vehicle,vehicle.heading) 
    vehicle.simple_goto(target)
    start = time.time()    
    while vehicle.mode.name=="GUIDED":
        current = time.time() - start
        dTarget = sqrt(pow(target.lat-vehicle.location.global_frame.lat,2)+pow(target.lon-vehicle.location.global_frame.lon,2)+pow(target.alt-vehicle.location.global_frame.alt,2))
        print " -> T: %0.1f, Alt: %0.1f, ToGo: %0.2f" % (current, vehicle.location.global_frame.alt, dTarget)
        if abs(vehicle.location.global_relative_frame.alt-target.alt) < 0.05: 
            print "\n\tReached %0.1f m in %0.1f sec!\n" % (target.alt, current)
            break
        time.sleep(0.5)


def go_to(vehicle, target):
    """
    Function that makes the vehicle travel to an specific lat/lon location. Measures distance and if the target is reached.
    """
    timeout = 20
    min_distance = 0.000005 # Parameter to tune by experimenting
    vehicle.simple_goto(target)
    start = time.time()    
    while vehicle.mode.name=="GUIDED":
        current = time.time() - start
        dTarget = sqrt(pow(target.lat-vehicle.location.global_frame.lat,2)+pow(target.lon-vehicle.location.global_frame.lon,2)++pow(target.alt-vehicle.location.global_frame.alt,2))
        print " ->T:%0.1f, Target[%0.2f %0.2f %0.1f], Actual[%0.2f %0.2f %0.1f], ToGo:%0.6f" % (current, target.lat, target.lon, target.alt, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt, dTarget)
        if dTarget<=min_distance:
            print "Reached target location"
            break;
        if current >= timeout:
            print "Timeout to reach location, last distance: %0.4f" % (dTarget)
            break;
        time.sleep(0.5)


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def condition_yaw(vehicle, heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def move_servo(vehicle, port, value):
    """
    Function that moves a servo from a specified port and value
    port  -> port where the servo is attached
    value -> servo ms value, from 1000 - 2000
    """
    msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, port, value, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)


def goto_position_target_local_ned(vehicle, north, east, down):
    """ 
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.
    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    #for x in range(0,duration):
    vehicle.send_mavlink(msg)
    #time.sleep(duration)

def send_global_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    #for x in range(0,duration):
    vehicle.send_mavlink(msg)
    #time.sleep(1)    

    