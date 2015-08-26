#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-joystick.py: Send joystick commands via UDP from a ground-station running Matlab to a FC running MultiWii software."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"
__video__ = "http://www.youtube.com/watch?v=XyyfGp-IomE"

import time, threading
from modules.pyMultiwii import MultiWii
import modules.UDPserver 

# RC commnads to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000,1000,1000,1000,1000]

# MRUAV initialization
vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")
#vehicle = MultiWii("/dev/ttyUSB0")

# Function to update commands and attitude to be called by a thread
def updateCMDATT():
    global vehicle, rcCMD
    try:
        while True:
            if modules.UDPserver.active:
                # Part for applying commands to the vehicle.
                rcCMD[0] = modules.UDPserver.message[0]
                rcCMD[1] = modules.UDPserver.message[1]
                rcCMD[2] = modules.UDPserver.message[2]
                rcCMD[3] = modules.UDPserver.message[3]
                vehicle.sendCMDreceiveATT(16,MultiWii.SET_RAW_RC,rcCMD)
                #print vehicle.attitude
                #print modules.UDPserver.message
                #time.sleep(0.01)
            #else: 
                # Part for landing and disarming.
                #print modules.UDPserver.message
                #print "Landing!"
                #time.sleep(1)
    except Exception,error:
        print "Error on updateCMDATT thread: "+str(error)
        updateCMDATT()

if __name__ == "__main__":
    try:
        vehicleThread = threading.Thread(target=updateCMDATT)
        vehicleThread.daemon=True
        vehicleThread.start()
        modules.UDPserver.startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()