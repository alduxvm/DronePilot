#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-simulink-controller.py -> Override RC commands with those of coming via UDP from a position-controller software done in Simulink. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"
__video__ = ""

import time, threading
'''  To import own modules, you need to export the current path before importing the module.    '''
'''  This also means that mavproxy must be called inside the folder of the script to be called. ''' 
import os, sys
sys.path.append(os.getcwd())
import modules.UDPserver as udp
import modules.utils as utils
import modules.pixVehicle

api = local_connect()
vehicle = api.get_vehicles()[0]

def sendCommands():
    """
    Function to update commands and attitude to be called by a thread.
    """
    try:
        while True:
            if udp.active:
                # Timers
                current = time.time()
                elapsed = 0
                
                # Part for applying commands to the vehicle.
                # Channel order in mavlink:   roll, pitch, throttle, yaw
                # Channel order in optitrack: roll, pitch, yaw, throttle

                # Joystick manual commands:
                roll     = udp.message[0]
                pitch    = utils.mapping(udp.message[1],1000,2000,2000,1000) # To invert channel, maybe add function
                yaw      = utils.mapping(udp.message[2],1000,2000,968,2062) # Map it to match RC configuration
                throttle = utils.mapping(udp.message[3],1000,2000,968,1998) # Map it to match RC configuration

                # Simulink automatic control:
                if udp.message[7] == 1:
                    roll     = udp.message[12]
                    pitch    = utils.mapping(udp.message[13],1000,2000,2000,1000) # To invert channel, maybe add function
                    yaw      = utils.mapping(udp.message[14],1000,2000,968,2062) # Map it to match RC configuration
                    throttle = utils.mapping(udp.message[15],1000,2000,968,1998) # Map it to match RC configuration

                # Vehicle communication
                vehicle.channel_override = { "1" : roll, "2" : pitch, "3" : throttle, "4" : yaw }
                vehicle.flush()
                
                # Debug
                #print "%s" % vehicle.attitude
                #print "%s" % vehicle.channel_readback
                print udp.message

                # 100hz loop
                while elapsed < 0.01:
                    elapsed = time.time() - current
                # End of the main loop
    except Exception,error:
        print "Error on sendCommands thread: "+str(error)
        sendCommands()

""" Section that starts the threads """
try:
    vehicleThread = threading.Thread(target=sendCommands)
    vehicleThread.daemon=True
    vehicleThread.start()
    udp.startTwisted()
except Exception,error:
    print "Error on main script thread: "+str(error)
