#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pixVehicle.py -> Module that contains several common functions for pixhawk vehicles. Updated to DroneKit 2.0 """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "2.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, math
from dronekit import connect, VehicleMode

def arm_and_takeoff(vehicle, aTargetAltitude):
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
	vehicle.mode    = VehicleMode("GUIDED")
	vehicle.armed   = True

	while not vehicle.armed:
		print "Waiting for arming..."
		time.sleep(1)

	print "Taking off!"
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	while True:
		print " Altitude: ", vehicle.location.global_relative_frame.alt
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Just below target, in case of undershoot.
			print "Reached target altitude"
			break;
		time.sleep(1)



def go_to(vehicle, target):
	"""
	Function that makes the vehicle travel to an specific lat/lon location. Measures distance and if the target is reached.
	"""
	timeout = 20
	min_distance = 0.000005 # Parameter to tune by experimenting
	vehicle.simple_goto(target)
	start = time.time()    
	while True:
		current = time.time() - start
		dTarget = math.sqrt(math.pow(target.lat-vehicle.location.global_frame.lat,2)+math.pow(target.lon-vehicle.location.global_frame.lon,2))
		print " ->%0.2f Travelling to WP, distance = %f" % (current, dTarget)
		if dTarget<=min_distance:
			print "Reached target location"
			break;
		if current >= timeout:
			print "Timeout to reach location, last distance: %0.4f" % (dTarget)
			break;
		time.sleep(0.5)

'''

def send_velocity_vector(velocity_x, velocity_y, velocity_z):
	"""
	Send a velocity vector for the vehicle to track.
	"""
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
	"""
	Set the heading into a specific value regardless goto functions.
	"""
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

def move_servo(port,value):
	"""
	Function that moves a servo from a specified port and value
	port  -> port where the servo is attached
	value -> servo ms value, from 1000 - 2000
	"""
	msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, port, value, 0, 0, 0, 0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
'''
