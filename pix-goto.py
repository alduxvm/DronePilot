#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-goto.py -> Script that commands the vehicle to follow waypoints. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "2.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import modules.UDPserver as udp
from modules.utils import *
from modules.pixVehicle import *

# Connection to the vehicle
# SITL via TCP
#vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
# SITL/vehicle via UDP (connection coming from mavproxy.py)
vehicle = connect('udp:127.0.0.1:14549', wait_ready=True)
# Direct UART communication to Pixhawk
#vehicle = connect('/dev/ttyAMA0', wait_ready=True)

""" Mission starts here """

arm_and_takeoff(vehicle, 10)

point1 = LocationGlobalRelative(55.870586,-4.287632, 25)
go_to(vehicle, point1)

point2 = LocationGlobalRelative(55.870548,-4.287313, 25)
go_to(vehicle, point2)

point3 = LocationGlobalRelative(55.870519,-4.287637, 25)
go_to(vehicle, point3)

point4 = LocationGlobalRelative(55.870576,-4.288043, 20)
go_to(vehicle, point4)

print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")

print "Waiting 10 seconds RTL"
time.sleep(10)

print "Landing the Aircraft"
vehicle.mode = VehicleMode("LAND")
