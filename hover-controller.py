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
from DroneModules.pid2 import pid

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
gains = {'kp':1.0, 'ki':0.5, 'kd':0.01, 'iMax':4}
#gains = {'kp':1.0, 'ki':0.5, 'kd':0.01}
# PID module 1
rollPID = PID(gains['kp'], gains['ki'], gains['kd'], 0, 0, gains['iMax'], gains['iMax'] * -1)
rollPID.setPoint(desiredPos['x'])
pitchPID = PID(gains['kp'], gains['ki'], gains['kd'], 0, 0, gains['iMax'], gains['iMax'] * -1)
pitchPID.setPoint(desiredPos['y'])
# PID module 2
rollPID2 = pid(gains['kp'], gains['ki'], gains['kd'], gains['iMax'])
pitchPID2 = pid(gains['kp'], gains['ki'], gains['kd'], gains['iMax'])

# Function to update position control, to be called by a thread
def control():
    global rollPID, pitchPID, rollPID2, pitchPID2
    global desiredPos, currentPos
    try:
        while True:
            if udp.active:
                # Order of the position from Optitrack is: X, Z, Y
                currentPos['x'] = udp.message[4]
                currentPos['y'] = udp.message[6]
                currentPos['z'] = udp.message[5]
                #print udp.message

                # PID update module 1 (we might need to invert x for y...)
                rPIDvalue = rollPID.update(currentPos['x'])
                pPIDvalue = pitchPID.update(currentPos['y'])

                # PID update module 2 (we might need to invert x for y...)
                rPIDvalue2 = rollPID2.get_pid(  currentPos['x'] - desiredPos['x'], 0.05)
                pPIDvalue2 = pitchPID2.get_pid( currentPos['y'] - desiredPos['y'], 0.05)

                # Check before flying that compass is calibrated
                sinYaw = math.sin(math.radians( utils.mapping(vehicle.attitude['heading'],-180,180,0,360) ))
                cosYaw = math.cos(math.radians( utils.mapping(vehicle.attitude['heading'],-180,180,0,360) ))

                desiredRoll  = utils.toPWM(math.degrees( (rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / utils.g) ))
                desiredPitch = utils.toPWM(math.degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / utils.g) ))

                desiredRoll2  = utils.toPWM(math.degrees( (rPIDvalue2 * sinYaw - pPIDvalue2 * cosYaw) * (1 / utils.g) ))
                desiredPitch2 = utils.toPWM(math.degrees( (rPIDvalue2 * cosYaw + pPIDvalue2 * sinYaw) * (1 / utils.g) ))

                print "PID 1: x = %d dRoll = %d | y = %d dPitch = %d" % (currentPos['x'],desiredRoll,currentPos['y'],desiredPitch)
                print "PID 2: x = %d dRoll = %d | y = %d dPitch = %d\n" % (currentPos['y'],desiredRoll2,currentPos['y'],desiredPitch2)

            else:
                # Nothing to do but reset the rcCMD and the integrators (perhaps...)
                rcCMD = [1500,1500,1500,1000]
                rollPID.resetIntegrator()
                pitchPID.resetIntegrator()
                rollPID2.reset_I()
                pitchPID2.reset_I()

    except Exception,error:
        print "Error in control thread: "+str(error)

# Function to update position control, to be called by a thread
def updateCommands():
    global vehicle, rcCMD
    try:
        while True:
            if udp.active:
                # Update positions and current commands
                rcCMD[0] = udp.message[0]
                rcCMD[1] = udp.message[1]
                rcCMD[2] = udp.message[2]
                rcCMD[3] = udp.message[3]

                if udp.message[7] == 1:
                    rcCMD[0] = desiredRoll
                    rcCMD[1] = desiredPitch

                rcCMD = [utils.limit(n,1000,2000) for n in rcCMD]
                
                vehicle.sendCMDreceiveATT(8,MultiWii.SET_RAW_RC,rcCMD)

    except Exception,error:
        print "Error in updateCommands thread: "+str(error)

if __name__ == "__main__":
    try:
        vehicleThread = threading.Thread(target=updateCommands)
        vehicleThread.daemon=True
        vehicleThread.start()
        controlThread = threading.Thread(target=control)
        controlThread.daemon=True
        controlThread.start()
        udp.startTwisted()
    except Exception,error:
        print "Error in main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()