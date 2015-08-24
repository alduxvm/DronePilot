#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" drop-packet-test.py -> Script with added function to command a servo in order to drop a packet. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__maintainer__ = "Kyle Brown"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, math
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
#from modules.vehicle import *

api = local_connect()
vehicle = api.get_vehicles()[0]

""" Functions to be implemented inside a module - todo """

def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""
	print "Basic pre-arm checks"
	# Don't let the user try to fly autopilot is booting
	if vehicle.mode.name == "INITIALISING":
		print "Waiting for vehicle to initialise"
		time.sleep(1)
	while vehicle.gps_0.fix_type < 2:
		print "Waiting for GPS...:", vehicle.gps_0.fix_type
		time.sleep(1)
		
	print "Arming motors"
	# Copter should arm in GUIDED mode
	vehicle.mode    = VehicleMode("GUIDED")
	vehicle.armed   = True
	vehicle.flush()

	while not vehicle.armed and not api.exit:
		print " Waiting for arming..."
		time.sleep(1)

	print "Taking off!"
	vehicle.commands.takeoff(aTargetAltitude) # Take off to target altitude
	vehicle.flush()

	while not api.exit:
		print " Altitude: ", vehicle.location.alt
		if vehicle.location.alt>=aTargetAltitude*0.95: #Just below target, in case of undershoot.
			print "Reached target altitude"
			break;
		time.sleep(1)

def go_to(target):
	timeout = 20
	start = time.time()
	vehicle.commands.goto(target)
	vehicle.flush()
	
	while not api.exit:
			current = time.time() - start
			dTarget = math.sqrt(math.pow(target.lat-vehicle.location.lat,2)+math.pow(target.lon-vehicle.location.lon,2))
			print " ->%0.2f Traveling to WP, distance = %f" % (current, dTarget)
			if dTarget<=0.000005:
					print "Reached target location"
					break;
			if current >= timeout:
					print "Timeout to reach location"
					break;
			time.sleep(0.5)

def send_velocity_vector(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink.pde)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink.pde) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def condition_yaw(heading):
    msg = vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
            0,     # sequence
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
            2, # current - set to 2 to make it a guided command
            0, # auto continue
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0,          # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def drop_packet(port):
	vehicle = api.get_vehicles()[0]
	msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, port, 2000, 0, 0, 0, 0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
	time.sleep(3)
	print "Reseting Servo"
	msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, port, 1000, 0, 0, 0, 0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()



""" Mission starts here """

print "Dropping packet!"
drop_packet(6)
print"Packet dropped"
