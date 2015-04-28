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
#vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")
vehicle = MultiWii("/dev/ttyUSB0")

# Ask for current attitude to initialize attitude values
vehicle.getData(MultiWii.ATTITUDE)

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':0.0} # Set at the beginning (for now...)
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP

# Initialize RC commnads to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000]

# PID's initialization 
gains = {'kp':4.64, 'ki':1.13, 'kd':4.5, 'iMax':10}
#gains = {'kp':1.0, 'ki':0.5, 'kd':0.01}
# PID module 1
rollPID = PID(gains['kp'], gains['ki'], gains['kd'], 0, 0, gains['iMax'], gains['iMax'] * -1)
rollPID.setPoint(desiredPos['x'])
pitchPID = PID(gains['kp'], gains['ki'], gains['kd'], 0, 0, gains['iMax'], gains['iMax'] * -1)
pitchPID.setPoint(desiredPos['y'])
# PID module 2
#rollPID = pid(gains['kp'], gains['ki'], gains['kd'], gains['iMax'])
#pitchPID = pid(gains['kp'], gains['ki'], gains['kd'], gains['iMax'])
desiredRoll = 1500
desiredPitch = 1500

# Function to update position control, to be called by a thread
def control():
    global rollPID, pitchPID
    global desiredPos, currentPos
    global desiredRoll, desiredPitch
    try:
        while True:
            if udp.active:
                # Order of the position from Optitrack is: X, Z, Y
                currentPos['x'] = udp.message[5]
                currentPos['y'] = udp.message[4]
                currentPos['z'] = udp.message[6]
                #print udp.message

                # PID update module (we might need to invert x for y...)
                #rPIDvalue = rollPID.get_pid(  currentPos['x'] - desiredPos['x'], 0.05)
                #pPIDvalue = pitchPID.get_pid( currentPos['y'] - desiredPos['y'], 0.05)
                rPIDvalue = rollPID.update(currentPos['x'])
                pPIDvalue = pitchPID.update(currentPos['y'])

                # Check before flying that compass is calibrated
                sinYaw = math.sin(math.radians( vehicle.attitude['heading'] ))
                cosYaw = math.cos(math.radians( vehicle.attitude['heading'] ))

                #desiredRoll  = utils.toPWM(math.degrees( (rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / utils.g) ))
                #desiredPitch = utils.toPWM(math.degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / utils.g) ))

                # Mellinger paper
                #desiredRoll  = utils.toPWM(math.degrees( (rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / utils.g) ),1)
                #desiredPitch = utils.toPWM(math.degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / utils.g) ),1)

                # Murray 
                desiredRoll  = utils.toPWM(math.degrees( (rPIDvalue * cosYaw - pPIDvalue * sinYaw) * (1 / utils.g) ),1)
                desiredPitch = utils.toPWM(math.degrees( (pPIDvalue * cosYaw - rPIDvalue * sinYaw) * (1 / utils.g) ),1)

                print "x = %0.2f Roll = %d y = %0.2f Pitch = %d yaw= %0.2f" % (currentPos['y'],desiredRoll,currentPos['y'],desiredPitch,math.radians(vehicle.attitude['heading']))
            else:
                # Nothing to do but reset the rcCMD and the integrators (perhaps...)
                rcCMD = [1500,1500,1500,1000]
                #rollPID.resetIntegrator()
                #pitchPID.resetIntegrator()
                rollPID.reset_I()
                pitchPID.reset_I()

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