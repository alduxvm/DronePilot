#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-simulink-controller.py: Override RC commands with those of coming via UDP from a position-controller software done in Simulink."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"
__video__ = ""

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
                # UDP message order:
                # Jroll, Jpitch, Jyaw, Jthrottle, X, Y, Z, Button, roll, pitch, yaw, NotUsed, RCroll, RCpitch, RCyaw, RCthrottle, RCaux1, RCaux2, RCaux3, RCaux4, NotUsed
                
                # Joystick manual commands
                rcCMD[0] = udp.message[0] # Roll
                rcCMD[1] = udp.message[1] # Pitch
                rcCMD[2] = udp.message[2] # Yaw
                rcCMD[3] = udp.message[3] # Throttle

                # If joystick button is on green (activated) then simulink is in charge. TODO: Check order of channels.
                if udp.message[7] == 1:
                    rcCMD[0] = udp.message[12] # Roll
                    rcCMD[1] = udp.message[13] # Pitch
                    rcCMD[2] = udp.message[14] # Yaw
                    rcCMD[3] = udp.message[15] # Throttle
                
                vehicle.sendCMD(16,MultiWii.SET_RAW_RC,rcCMD)
                print udp.message
                time.sleep(0.0125) # 80 hz
            #else: 
                # Part for landing and disarming.
                #print modules.UDPserver.message
                #print "Landing!"
                #time.sleep(1)
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