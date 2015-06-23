#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-logdata.py -> Script that logs data from a computer vision thread and from a flying multicopter with a Pixhawk. DroneApi related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__maintainer__ = "Kyle Brown"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

# Dependancies
# * This script assumes that the pixhawk is connected to the raspeberry pi via the serial port (/dev/ttyAMA0)
# * In our setup, telemetry port 2 is configured at 115200 on the pixhawk
# * rpi camera connected (for computer vision)

# Usage:
# * mavproxy.py --master=/dev/ttyAMA0 --baudrate 115200 --aircraft testQuad
# * module load api
# * api start pix-logdata.py

from droneapi.lib import VehicleMode
from pymavlink import mavutil
import time, threading, csv, datetime

#from modules.vision import ColorTracker 

# Vehicle initialisation
api = local_connect()
vehicle = api.get_vehicles()[0]

# Waiting until the vehicle initialise
c=0
while vehicle.mode.name is 'INITIALISING':
	print "Waiting for vehicle to initialise %d" % (c)
	c+=1
	time.sleep(1)

# Computer vision initialization 
#landingPad = ColorTracker('red')
#visionThread = threading.Thread(target=test.findColor)
#visionThread.daemon=True
#visionThread.start()

# Logger initialization 
st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
f = open("logs/"+st, "w")
logger = csv.writer(f)
logger.writerow(('timestamp','angx','angy','heading','battery','mode','longitude','latitude','altitude','targetX','targetY'))

while True:
#	logger.writerow((vehicle.attitude['timestamp'],vehicle.attitude['elapsed'], \
#                     vehicle.attitude['angx'],vehicle.attitude['angy'],vehicle.attitude['heading'], \
#                     currentPos['x'],currentPos['y'],currentPos['z'], \
#                     rcCMD[0],rcCMD[1],rcCMD[2],rcCMD[3], \
#                     rPIDvalue,pPIDvalue )) 
	print "%s" % vehicle.attitude
	time.sleep(0.05)








