#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" modules/vehicle.py -> Module that handles several functions related to the vehicle using DroneAPI """

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


def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to fly autopilot is booting
    if self.mode.name == "INITIALISING":
        print "Waiting for vehicle to initialise"
        time.sleep(1)
    while self.gps_0.fix_type < 2:
        print "Waiting for GPS...:", self.gps_0.fix_type
        time.sleep(1)
		
    print "Arming motors"
    # Copter should arm in GUIDED mode
    self.mode    = selfMode("GUIDED")
    self.armed   = True
    self.flush()

    while not self.armed and not api.exit:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    self.commands.takeoff(aTargetAltitude) # Take off to target altitude
    self.flush()

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.commands.takeoff will execute immediately).
    while not api.exit:
        print " Altitude: ", self.location.alt
        if self.location.alt>=aTargetAltitude*0.95: #Just below target, in case of undershoot.
            print "Reached target altitude"
            break;
        time.sleep(1)
