#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-showdata.py -> Script that shows data from a vehicle. DroneKit 2.0 related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "2.0"
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
# SITL via UDP 
vehicle = connect('udp:127.0.0.1:14549', wait_ready=True)
# Real vehicle via Serial Port 
#vehicle = connect('/dev/tty.usbmodem1', wait_ready=False)

while True:
	print "%s" % vehicle.attitude #SR2_EXTRA1
	print "%s" % vehicle.velocity #SR2_POSITION
	print "%s" % vehicle.channels #SR2_RC_CHAN
	print "Altitude (global frame): %s" % vehicle.location.global_frame.alt
	print "Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt
	print "%s" % vehicle.mode.name
	#print "%s" % udp.message
	time.sleep(0.01)

vehicle.close()

''' 
param set SR2_EXTRA1 50
param set SR2_POSITION 50
param set SR2_RAW_CTRL 50
param set SR2_RC_CHAN 50

''' 
