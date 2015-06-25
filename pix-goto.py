#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-goto.py -> Script that commands the vehicle to follow waypoints. """

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



""" Mission starts here """

arm_and_takeoff(10)

point1 = Location(55.870586,-4.287632, 25, is_relative=True)
go_to(point1)

point2 = Location(55.870548,-4.287313, 25, is_relative=True)
go_to(point2)

point3 = Location(55.870519,-4.287637, 25, is_relative=True)
go_to(point3)

print "Returning to Launch"
vehicle.mode    = VehicleMode("RTL")
vehicle.flush()
print "Waiting 10 seconds RTL"
time.sleep(10)

print "Landing the Aircraft"
vehicle.mode    = VehicleMode("LAND")
vehicle.flush()