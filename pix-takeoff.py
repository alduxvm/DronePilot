#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-takeoff.py -> Script that makes a pixhawk take off in the most secure way. DroneApi related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__maintainer__ = "Kyle Brown"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time
from droneapi.lib import VehicleMode
from pymavlink import mavutil

api = local_connect()
vehicle = api.get_vehicles()[0]

def arm_and_takeoff(aTargetAltitude):
	
	print "Basic pre-arm checks"
		# Don't let the user try to fly autopilot is booting
		# Waiting until the vehicle initialise
		# Waiting until the vehicle initialise
		c=0
		if vehicle.mode.name == 'INITIALISING':
			print "Waiting for vehicle to initialise %d" % (c)
				c+=1
				time.sleep(1)
		time.sleep(4)
		# Waiting for GPS fix
		while vehicle.gps_0.fix_type < 2:
			print "Waiting for GPS...:", vehicle.gps_0.fix_type
				time.sleep(1)
				break
	
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
		
		# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
		#  after Vehicle.commands.takeoff will execute immediately).
		while not api.exit:
			print " Altitude: ", vehicle.location.alt
				time.sleep(1)
				if vehicle.location.alt>=aTargetAltitude*0.95: #Just below target, in case of undershoot.
					print "Reached target altitude"
						break;
		time.sleep(1)

arm_and_takeoff(3)


time.sleep(10)
print "Landing the Aircraft"
vehicle.mode    = VehicleMode("LAND")
vehicle.flush()