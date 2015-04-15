#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" position-control.py: Send commands to a MultiWii vehicle to keep a certain position with information from a MoCap system."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, threading, math
from DroneModules.pyMultiwii import MultiWii
import DroneModules.UDPserver 
from DroneModules.pid import PID

# MRUAV initialization
vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")
#vehicle = MultiWii("/dev/ttyUSB0")

# Ask for current attitude
vehicle.getData(MultiWii.ATTITUDE)

# Position coordinates [x, y, x] 
desiredPos = [0, 0, 0] # Set at the beginning (for now...)
vehiclePos = [0, 0, 0] # It will be updated using UDP

# RC commnads to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000,1000,1000,1000,1000]

# PID's initialization 
rollPID = PID(3.0,0.4,1.2)
rollPID.setPoint(desiredPos[0])

pitchPID = PID(3.0,0.4,1.2)
pitchPID.setPoint(desiredPos[1])

# Function to map a value to another 
def toPWM(value):
    iMin = -50
    iMax = 50
    oMin = 1000
    oMax = 2000
    return round((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin, 0)

# Function to update position control, to be called by a thread
def position_control():
    global vehicle, rcCMD, rollPID, pitchPID
    try:
        while True:
            if DroneModules.UDPserver.active:
                # Update positions and current commands
                rcCMD[0] = DroneModules.UDPserver.message[0]
                rcCMD[1] = DroneModules.UDPserver.message[1]
                rcCMD[2] = DroneModules.UDPserver.message[2]
                rcCMD[3] = DroneModules.UDPserver.message[3]
                vehiclePos[0] = DroneModules.UDPserver.message[4]
                vehiclePos[1] = DroneModules.UDPserver.message[5]
                vehiclePos[2] = DroneModules.UDPserver.message[6]

                rPIDvalue = rollPID.update(vehiclePos[0])
                pPIDvalue = pitchPID.update(vehiclePos[1])

                yawSin = math.sin(math.radians(vehicle.attitude['heading']))
                yawCos = math.cos(math.radians(vehicle.attitude['heading']))

                newRoll = toPWM(math.degrees(yawSin * rPIDvalue - yawCos * pPIDvalue))
                newPitch = toPWM(math.degrees(yawCos * rPIDvalue - yawCos * pPIDvalue))

                print "Roll value = %d  Pitch value = %d" % (newRoll,newPitch)
                vehicle.sendCMDreceiveATT(16,MultiWii.SET_RAW_RC,rcCMD)
                time.sleep(1)
                #print vehicle.attitude
                #print DroneModules.UDPserver.message
                #time.sleep(0.01)
            #else: 
                # Part for landing and disarming.
                #print DroneModules.UDPserver.message
                #print "Landing!"
                #time.sleep(1)
    except Exception,error:
        print "Error on position_control thread: "+str(error)
        position_control()

if __name__ == "__main__":
    try:
        vehicleThread = threading.Thread(target=position_control)
        vehicleThread.daemon=True
        vehicleThread.start()
        DroneModules.UDPserver.startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()