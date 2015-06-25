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
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil

api = local_connect()
vehicle = api.get_vehicles()[0]

""" Functions to be implemented inside a module - todo """

def arm_and_takeoff(targetAltitude):
    """
    Arms vehicle and fly to a target altitude.
    """

    print "Basic pre-arm checks"
    if vehicle.mode.name == "INITIALISING":
        print "Waiting for vehicle to initialise"
        time.sleep(1)
    while vehicle.gps_0.fix_type < 2:
        print "Waiting for GPS...:", vehicle.gps_0.fix_type
        time.sleep(1)
		
    print "Arming motors"
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    vehicle.flush()

    while not vehicle.armed and not api.exit:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.commands.takeoff(targetAltitude) # Take off to target altitude
    vehicle.flush()

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.commands.takeoff will execute immediately).
    while not api.exit:
        print " Altitude: ", vehicle.location.alt
        if vehicle.location.alt>=targetAltitude*0.95: #Just below target, in case of undershoot.
            print "Reached target altitude"
            break;
        time.sleep(1)



""" Mission starts here """

arm_and_takeoff(3)

time.sleep(10)
print "Landing the Aircraft"
vehicle.mode    = VehicleMode("LAND")
vehicle.flush()