#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" hover-controller.py: Send commands to a MultiWii vehicle to keep a certain position with information from a MoCap system."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, threading, math
from DroneModules.pyMultiwii import MultiWii
import DroneModules.utils as utils
import DroneModules.UDPserver as udp
from DroneModules.pid import PID

# MRUAV initialization
vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")
#vehicle = MultiWii("/dev/ttyUSB0")

# Ask for current attitude to initialize attitude values
vehicle.getData(MultiWii.ATTITUDE)

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':0.0} # Set at the beginning (for now...)
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP

# Initialize RC commnads to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000]

# PID's initialization 
rollPID = PID(3.0, 0.4, 1.2)
rollPID.setPoint(desiredPos['x'])

pitchPID = PID(3.0, 0.4, 1.2)
pitchPID.setPoint(desiredPos['y'])

# Function to update position control, to be called by a thread
def hover():
    global vehicle, rcCMD, rollPID, pitchPID
    try:
        while True:
            if udp.active:
                # Update positions and current commands
                rcCMD[0] = udp.message[0]
                rcCMD[1] = udp.message[1]
                rcCMD[2] = udp.message[2]
                rcCMD[3] = udp.message[3]
                # Order of the position from Optitrack is: X, Z, Y
                currentPos['x'] = udp.message[4]
                currentPos['y'] = udp.message[6]
                currentPos['z'] = udp.message[5]
                #print udp.message

                rPIDvalue = rollPID.update(currentPos['x'])
                pPIDvalue = pitchPID.update(currentPos['y'])

                # Check this and calibrate compass
                sinYaw = math.sin(math.radians( utils.mapping(vehicle.attitude['heading'],-180,180,0,360) ))
                cosYaw = math.cos(math.radians( utils.mapping(vehicle.attitude['heading'],-180,180,0,360) ))

                # Check where to do the degrees convertion
                desiredRoll  = utils.toPWM(math.degrees( (rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / utils.g) ))
                desiredPitch = utils.toPWM(math.degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / utils.g) ))

                print "x = %d desired Roll = %d" % (currentPos['x'],desiredRoll)
                print "y = %d desired Pitch = %d" % (currentPos['x'],desiredPitch)
                vehicle.sendCMDreceiveATT(8,MultiWii.SET_RAW_RC,rcCMD)
                time.sleep(1)

    except Exception,error:
        print "Error on position_control thread: "+str(error)
        position_control()

if __name__ == "__main__":
    try:
        vehicleThread = threading.Thread(target=hover)
        vehicleThread.daemon=True
        vehicleThread.start()
        udp.startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()