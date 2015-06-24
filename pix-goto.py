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

import time
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
from modules.vehicle import vc

api = local_connect()
vehicle = api.get_vehicles()[0]

vc.arm_and_takeoff(vehicle,20)

print "Going to first point..."
point1 = Location(-35.361354, 149.165218, 20, is_relative=True)
vehicle.commands.goto(point1)
vehicle.flush()

# sleep so we can see the change in map
time.sleep(30)

print "Going to second point..."
point2 = Location(-35.363244, 149.168801, 20, is_relative=True)
vehicle.commands.goto(point2)
vehicle.flush()

# sleep so we can see the change in map
time.sleep(20)

print "Returning to Launch"
vehicle.mode    = VehicleMode("RTL")
vehicle.flush()