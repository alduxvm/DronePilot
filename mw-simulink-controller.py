#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-simulink-controller.py: Override RC commands with those of coming via UDP from a position-controller software done in Simulink."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1.6"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"
__video__ = ""

import time, threading, datetime, csv
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
        # Open file to start logging
        st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
        f = open("logs/mw-simulink-"+st, "w")
        logger = csv.writer(f)
        #logger.writerow(('timestamp','ax','ay','az','gx','gy','gz','x','y','z','attx','atty','attz','Sroll','Spitch','Syaw','Sthrottle'))
        logger.writerow(('timestamp','angx','angy','heading','x','y','z','attx','atty','attz','Sroll','Spitch','Syaw','Sthrottle'))
        while True:
            if udp.active:
                # Timers
                current = time.time()
                elapsed = 0

                # UDP message order:
                # Jroll, Jpitch, Jyaw, Jthrottle, X, Y, Z, Button, roll, pitch, yaw, NotUsed, RCroll, RCpitch, RCyaw, RCthrottle, RCaux1, RCaux2, RCaux3, RCaux4, NotUsed
                
                # Joystick manual commands
                rcCMD[0] = udp.message[0] # Roll
                rcCMD[1] = udp.message[1] # Pitch
                rcCMD[2] = udp.message[2] # Yaw
                rcCMD[3] = udp.message[3] # Throttle

                # If joystick button is on green (activated) then simulink is in charge. 
                if udp.message[7] == 1:
                    rcCMD[0] = udp.message[12] # Roll
                    rcCMD[1] = udp.message[13] # Pitch
                    rcCMD[2] = udp.message[14] # Yaw
                    rcCMD[3] = udp.message[15] # Throttle
                
                # Vehicle communication
                vehicle.sendCMD(16,MultiWii.SET_RAW_RC,rcCMD)
                #time.sleep(0.005) # Apparently not needed, leaved just in case.
                vehicle.getData(MultiWii.ATTITUDE)

                row =   (current, \
                        vehicle.attitude['angx'], vehicle.attitude['angy'], vehicle.attitude['heading'], \
                        #vehicle.rawIMU['ax'], vehicle.rawIMU['ay'], vehicle.rawIMU['az'], vehicle.rawIMU['gx'], vehicle.rawIMU['gy'], vehicle.rawIMU['gz'], \
                        #vehicle.rcChannels['roll'], vehicle.rcChannels['pitch'], vehicle.rcChannels['throttle'], vehicle.rcChannels['yaw'], \
                        #udp.message[0], udp.message[1], udp.message[3], udp.message[2], \
                        udp.message[4], udp.message[5], udp.message[6], \
                        udp.message[8], udp.message[9], udp.message[10], \
                        udp.message[12], udp.message[13], udp.message[14], udp.message[15])
                logger.writerow(row)

                # 100hz loop
                time.sleep(0.01)
                #while elapsed < 0.01:
                #    elapsed = time.time() - current
                print udp.message
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