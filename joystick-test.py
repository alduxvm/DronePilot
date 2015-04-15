#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
"""joystick-test.py: Send commands coming from a Joystick via UDP using a Ground station with Matlab."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, threading
from DroneModules.pyMultiwii import MultiWii
import DroneModules.UDPserver 

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
            if DroneModules.UDPserver.active:
                # Part for applying commands to the vehicle.
                rcCMD[0] = DroneModules.UDPserver.message[0]
                rcCMD[1] = DroneModules.UDPserver.message[1]
                rcCMD[2] = DroneModules.UDPserver.message[2]
                rcCMD[3] = DroneModules.UDPserver.message[3]
                vehicle.sendCMDreceiveATT(16,MultiWii.SET_RAW_RC,rcCMD)
                #print vehicle.attitude
                #print DroneModules.UDPserver.message
                #time.sleep(0.01)
            #else: 
                # Part for landing and disarming.
                #print DroneModules.UDPserver.message
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
        DroneModules.UDPserver.startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()