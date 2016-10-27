#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-joystick.py: Send joystick commands via UDP from a ground-station running Matlab to a FC running MultiWii software."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "1.6"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"
__video__ = "http://www.youtube.com/watch?v=XyyfGp-IomE"

import time, threading
from modules.pyMultiwii import MultiWii
import modules.UDPserver as udp

# RC commnads to be sent to the MultiWii 
# Order: roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4
rcCMD = [1500,1500,1500,1000,1000,1000,1000,1000]

# MRUAV initialization
#vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")
vehicle = MultiWii("/dev/ttyUSB0")

# Function to update commands and attitude to be called by a thread
def sendCommands():
    global vehicle, rcCMD
    try:
        while True:
            if udp.active:
                current = time.time()
                elapsed = 0
                # Part for applying commands to the vehicle.
                # Joystick manual commands
                rcCMD[0] = udp.message[0] # Roll
                rcCMD[1] = udp.message[1] # Pitch
                rcCMD[2] = udp.message[2] # Yaw
                rcCMD[3] = udp.message[3] # Throttle
                vehicle.sendCMD(16,MultiWii.SET_RAW_RC,rcCMD)
                print udp.message # Just to display some data, not really needed.
                # 100hz loop
                while elapsed < 0.01:
                    elapsed = time.time() - current
                # End of the main loop
    except Exception,error:
        print "Error on sendCommands thread: "+str(error)
        sendCommands()

if __name__ == "__main__":
    try:
        vehicleThread = threading.Thread(target=sendCommands)
        vehicleThread.daemon=True
        vehicleThread.start()
        udp.startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()