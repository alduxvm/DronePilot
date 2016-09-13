#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-velocity-vector.py -> Script that send the vehicle a velocity vector to form a square and diamond shape. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time
from dronekit import connect, VehicleMode
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

NORTH=2
SOUTH=-2
EAST=2
WEST=-2
UP=-0.5
DOWN=0.5

DURATION=20

# Shape shape
print "Making a square!"

condition_yaw(vehicle,0)
send_ned_velocity(vehicle,NORTH,0,0,DURATION)
print "Flying for 20 seconds direction NORTH!"
#send_ned_velocity(vehicle,0,0,0,5)

condition_yaw(vehicle,90)
send_ned_velocity(vehicle,0,EAST,0,DURATION)
print "Flying for 20 seconds direction EAST!"
#send_ned_velocity(vehicle,0,0,0,5)

condition_yaw(vehicle,180)
send_ned_velocity(vehicle,SOUTH,0,0,DURATION)
print "Flying for 20 seconds direction SOUTH!"
#send_ned_velocity(vehicle,0,0,0,5)

condition_yaw(vehicle,270)
send_ned_velocity(vehicle,0,WEST,0,DURATION)
print "Flying for 20 seconds direction WEST!"
#send_ned_velocity(vehicle,0,0,0,5)

# Diamond shape
print "Making a diamond!"

print("Going North, East and up")
condition_yaw(vehicle,90)
send_ned_velocity(vehicle,NORTH,EAST,UP,DURATION)

print("Going South, East and down")
condition_yaw(vehicle,90)
send_ned_velocity(vehicle,SOUTH,EAST,DOWN,DURATION)

print("Going South and West")
condition_yaw(vehicle,90)
send_ned_velocity(vehicle,SOUTH,WEST,0,DURATION)

print("Going North and West")
condition_yaw(vehicle,90)
send_ned_velocity(vehicle,NORTH,WEST,0,DURATION)


print "Returning to Launch"
vehicle.mode    = VehicleMode("RTL")
print "Waiting 10 seconds RTL"
time.sleep(10)

print "Landing the Aircraft"
vehicle.mode    = VehicleMode("LAND")
