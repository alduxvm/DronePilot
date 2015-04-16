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
gains = {'kp':3.0, 'ki':0.4, 'kd':1.2, 'iMax':50}
#gains = {'kp':1.0, 'ki':0.5, 'kd':0.01}

# PID module 1
rollPID = PID(gains['kp'], gains['ki'], gains['kd'], gains['iMax'], gains['iMax'] * -1)
rollPID.setPoint(desiredPos['x'])
pitchPID = PID(gains['kp'], gains['ki'], gains['kd'], gains['iMax'], gains['iMax'] * -1)
pitchPID.setPoint(desiredPos['y'])

# PID module 2
rollPID2 = pid(gains['kp'], gains['ki'], gains['kd'], gains['iMax'])
pitchPID2 = pid(gains['kp'], gains['ki'], gains['kd'], gains['iMax'])

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

                # PID update module 1
                rPIDvalue = rollPID.update(currentPos['x'])
                pPIDvalue = pitchPID.update(currentPos['y'])

                # PID update module 2
                errorX = currentPos['x'] - desiredPos['x']
                errorY = currentPos['y'] - desiredPos['y']
                rPIDvalue2 = rollPID2.get_p(errorX) + rollPID2.get_i(errorX,0.1) + rollPID2.get_d(errorX,0.1)
                pPIDvalue2 = pitchPID2.get_p(errorY) + pitchPID2.get_i(errorY,0.1) + pitchPID2.get_d(errorY,0.1)

                # Check this and calibrate compass
                sinYaw = math.sin(math.radians( utils.mapping(vehicle.attitude['heading'],-180,180,0,360) ))
                cosYaw = math.cos(math.radians( utils.mapping(vehicle.attitude['heading'],-180,180,0,360) ))

                # Check where to do the degrees convertion
                desiredRoll  = utils.toPWM(math.degrees( (rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / utils.g) ))
                desiredPitch = utils.toPWM(math.degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / utils.g) ))

                desiredRoll2  = utils.toPWM(math.degrees( (rPIDvalue2 * sinYaw - pPIDvalue2 * cosYaw) * (1 / utils.g) ))
                desiredPitch2 = utils.toPWM(math.degrees( (rPIDvalue2 * cosYaw + pPIDvalue2 * sinYaw) * (1 / utils.g) ))

                print "PID 1: x = %d dRoll = %d | y = %d dPitch = %d" % (currentPos['x'],desiredRoll,currentPos['x'],desiredPitch)
                print "PID 2: x = %d dRoll = %d | y = %d dPitch = %d\n" % (currentPos['x'],desiredRoll2,currentPos['x'],desiredPitch2)
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