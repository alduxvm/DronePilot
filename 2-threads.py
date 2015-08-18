#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-joystick.py -> Script that send the vehicle joystick override using data from a UDP server. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"

__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, math, threading
import os, sys
sys.path.append(os.getcwd())
import modules.UDPserver

# Function to update commands and attitude to be called by a thread
def joystick():
    try:
        while True:
            if modules.UDPserver.active:
                # Part for applying commands to the vehicle.
                #vehicle.channel_override = { "1" : modules.UDPserver.message[0], "2" : modules.UDPserver.message[1], \
                #                             "3" : modules.UDPserver.message[2], "4" : modules.UDPserver.message[3] }
                #vehicle.flush()
                print modules.UDPserver.message
                time.sleep(0.01) # Maybe not needed?
            else:
            	print "No UDP active!"
                time.sleep(0.01)
    except Exception,error:
        print "Error on joystick thread: "+str(error)
        joystick()



try:
    vehicleThread = threading.Thread(target=joystick)
    vehicleThread.daemon=True
    vehicleThread.start()
    modules.UDPserver.startTwisted()
except Exception,error:
    print "Error on main: "+str(error)