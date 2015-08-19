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


import time, threading

'''  To import own modules, you need to export the current path before importing the module.    '''
'''  This also means that mavproxy must be called inside the folder of the script to be called. ''' 
import os, sys
sys.path.append(os.getcwd())
import modules.UDPserver
import modules.utils as utils
import modules.pixVehicle

api = local_connect()
vehicle = api.get_vehicles()[0]


def joystick():
    """
    Function to update commands and attitude to be called by a thread.
    """
    try:
        while True:
            if modules.UDPserver.active:
                # Part for applying commands to the vehicle.
                # Channel order in mavlink:   roll, pitch, throttle, yaw
                # Channel order in optitrack: roll, pitch, yaw, throttle
                roll     = modules.UDPserver.message[0]
                pitch    = utils.mapping(modules.UDPserver.message[1],1000,2000,2000,1000) # To invert channel, maybe add function
                throttle = utils.mapping(modules.UDPserver.message[3],1000,2000,968,1998) # Map it to match RC configuration
                yaw      = utils.mapping(modules.UDPserver.message[2],1000,2000,968,2062) # Map it to match RC configuration
                vehicle.channel_override = { "1" : roll, "2" : pitch, "3" : throttle, "4" : yaw }
                vehicle.flush()
                #print "%s" % vehicle.attitude
                print "%s" % vehicle.channel_readback
                #print modules.UDPserver.message
                time.sleep(0.009) # Maybe not needed?
    except Exception,error:
        print "Error on joystick thread: "+str(error)
        joystick()

""" Section that starts the threads """
try:
    vehicleThread = threading.Thread(target=joystick)
    vehicleThread.daemon=True
    vehicleThread.start()
    modules.UDPserver.startTwisted()
except Exception,error:
    print "Error on main script thread: "+str(error)
