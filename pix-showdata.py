#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-showdata.py -> Script that shows data from a vehicle and a MoCap system. DroneApi related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__maintainer__ = "Kyle Brown"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, threading, csv, datetime
'''  To import own modules, you need to export the current path before importing the module.    '''
'''  This also means that mavproxy must be called inside the folder of the script to be called. ''' 
import os, sys
sys.path.append(os.getcwd())
import modules.UDPserver as udp
import modules.utils as utils
import modules.pixVehicle

# Vehicle initialization
api = local_connect()
vehicle = api.get_vehicles()[0]

""" Section that starts the script """
 while True:
    print "%s" % vehicle.attitude #SR2_EXTRA1
    print "%s" % vehicle.velocity #SR2_POSITION
    print "%s" % vehicle.channel_readback #SR2_RC_CHAN
    print "%s" % udp.message
    time.sleep(0.05)

''' 
param set SR2_EXTRA1 10
param set SR2_POSITION 10
param set SR2_RAW_CTRL 10
param set SR2_RC_CHAN 10

''' 