#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-logdata.py -> Script that logs data from a vehicle and a MoCap system. DroneApi related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__maintainer__ = "Kyle Brown"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

# Dependencies
# * This script assumes that the pixhawk is connected to the raspberry pi via the serial port (/dev/ttyAMA0)
# * In our setup, telemetry port 2 is configured at 115200 on the pixhawk

# Usage:
# * mavproxy.py --master=/dev/ttyAMA0 --baudrate 115200 --aircraft TestQuad
# * module load api
# * api start pix-logdata.py

import time, threading, csv, datetime

'''  To import own modules, you need to export the current path before importing the module.    '''
'''  This also means that mavproxy must be called inside the folder of the script to be called. ''' 
import os, sys
sys.path.append(os.getcwd())
import modules.UDPserver
import modules.utils as utils
import modules.pixVehicle

# Vehicle initialization
api = local_connect()
vehicle = api.get_vehicles()[0]

# Function to manage data, print it and save it in a log 
def logit():
    try:
        st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
        f = open("logs/"+st, "w")
        logger = csv.writer(f)
        logger.writerow(('timestamp','angx','angy','heading','x','y','z','rcR','rcP','rcY','rcT'))
        while True:
            # Manage data and create messages
            #messATT = "angx = {:+.2f} \t angy = {:+.2f} \t heading = {:+.2f} ".format(float(board.attitude['angx']),float(board.attitude['angy']),float(board.attitude['heading']))
            #messPOS = "x = {:+.2f} \t y = {:+.2f} \t z = {:+.2f} ".format(float(currentPos['x']),float(currentPos['y']),float(currentPos['z']))
            #messPOS = "rE = {:+.2f} \t pE = {:+.2f} \t rPWM = {:.0f} \t pPWM = {:.0f} ".format(float(rPIDvalue),float(pPIDvalue),float(desiredRoll),float(desiredPitch)) 
            #messLOG = str(board.attitude['timestamp']) + "," + str(board.attitude['elapsed']) + "," + \
            #          str(board.attitude['angx']) + "," + str(board.attitude['angy']) + "," + str(board.attitude['heading']) + "," + \
            #          str(currentPos['x']) + "," + str(currentPos['y']) + "," + str(currentPos['z']) + "," + \
            #          str(rcCMD[0]) + "," + str(rcCMD[1]) + "," + str(rcCMD[2]) + "," + str(rcCMD[3]) + "," + \
            #          str(rPIDvalue) + "," + str(pPIDvalue) 
            # Print message
            #print "x = %0.2f Roll = %d y = %0.2f Pitch = %d yaw= %0.2f" % (currentPos['x'],desiredRoll,currentPos['y'],desiredPitch,math.radians(vehicle.attitude['heading']))
            #print "(%0.2f, %0.2f, %0.2f) | (%0.2f, %0.2f) | (%d, %d)" % (currentPos['x'],currentPos['y'],currentPos['z'],(rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / utils.g), (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / utils.g), desiredRoll, desiredPitch) 
            # Save log
            
            #logger.writerow((vehicle.attitude['timestamp'],vehicle.attitude['elapsed'], \
            #                 vehicle.attitude['angx'],vehicle.attitude['angy'],vehicle.attitude['heading'], \
            #                 currentPos['x'],currentPos['y'],currentPos['z'], \
            #                 rcCMD[0],rcCMD[1],rcCMD[2],rcCMD[3], \
            #                 rPIDvalue,pPIDvalue ))
            time.sleep(0.01) # To record data at 100hz exactly
    except Exception,error:
        print "Error in logit thread: "+str(error)
        f.close()

while True:
	if vehicle.armed and modules.UDPserver.active:
		logit()
	else:
		print "Waiting for vehicle to be armed and/or UDP server to be active..."
		print "Attitude: %s" % vehicle.attitude
		print "Velocity: %s" % vehicle.velocity
		print "Battery: %s" % vehicle.battery
		print "RC: %s" % vehicle.channel_readback
		time.sleep(0.5)







