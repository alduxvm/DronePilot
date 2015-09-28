#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-hover-controller.py: Calculate commands to make a Multiwii multicopter hover over a specified x,y,z coordinate."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, datetime, threading, csv
from math import *

from modules.pyMultiwii import MultiWii
from modules.utils import *
import modules.UDPserver as udp
#from modules.pid import PID
from modules.pids import PID_Controller

# MRUAV initialization
#vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")
#vehicle = MultiWii("/dev/tty.SLAB_USBtoUART") 
vehicle = MultiWii("/dev/ttyUSB0")

# Ask for current attitude to initialize attitude values
vehicle.getData(MultiWii.ATTITUDE)

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':0.0} # Set at the beginning (for now...)
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP

# Initialize RC commands and pitch/roll to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000]
desiredRoll = 1500
desiredPitch = 1500

# PID's initialization 
gains = {'kp':1.0, 'ki':0.1, 'kd':0.01, 'iMax':1}
rPIDvalue = 0.0
pPIDvalue = 0.0
#gains = {'kp':1.0, 'ki':0.5, 'kd':0.01}
# PID module 1
#rollPID = PID(gains['kp'], gains['ki'], gains['kd'], 0, 0, gains['iMax'], gains['iMax'] * -1)
#rollPID.setPoint(desiredPos['x'])
#pitchPID = PID(gains['kp'], gains['ki'], gains['kd'], 0, 0, gains['iMax'], gains['iMax'] * -1)
#pitchPID.setPoint(desiredPos['y'])
# PID module 2
#rollPID = pid(gains['kp'], gains['ki'], gains['kd'], gains['iMax'])
#pitchPID = pid(gains['kp'], gains['ki'], gains['kd'], gains['iMax'])

# PID module 3
#rollPID = utils.PID(gains['kp'], gains['ki'], gains['kd'])
#pitchPID = utils.PID(gains['kp'], gains['ki'], gains['kd'])
#rollPID.setTarget(desiredPos['x'])
#rollPID.setTarget(desiredPos['y'])

# PID module pids
rollPID = PID_Controller(gains['kp'], gains['ki'], gains['kd'])
pitchPID = PID_Controller(gains['kp'], gains['ki'], gains['kd'])


# Function to update position control, to be called by a thread
def control():
    global vehicle, rcCMD
    global rollPID, pitchPID
    global desiredPos, currentPos
    global desiredRoll, desiredPitch
    global rPIDvalue, pPIDvalue
    try:
        while True:
            udp.active = True
            if udp.active:
                # Read joystick commands from the ground station
                rcCMD[0] = udp.message[0]
                rcCMD[1] = udp.message[1]
                rcCMD[2] = udp.message[2]
                rcCMD[3] = udp.message[3]
                # Order of the position from Optitrack is: X, Z, Y
                currentPos['x'] = udp.message[5]
                currentPos['y'] = udp.message[4]
                currentPos['z'] = udp.message[6]
                #print udp.message

                # PID update module (we might need to invert x for y...)
                # pid 1
                #rPIDvalue = rollPID.get_pid(  currentPos['x'] - desiredPos['x'], 0.05)
                #pPIDvalue = pitchPID.get_pid( currentPos['y'] - desiredPos['y'], 0.05)
                # pid 2
                #rPIDvalue = rollPID.update(currentPos['x'])
                #pPIDvalue = pitchPID.update(currentPos['y'])
                # pid 3
                #rPIDvalue = rollPID.step(currentPos['x'])
                #pPIDvalue = pitchPID.step(currentPos['y'])
                # pid 4
                rPIDvalue = rollPID.getCorrection(desiredPos['x'],currentPos['x'])
                pPIDvalue = pitchPID.getCorrection(desiredPos['y'],currentPos['y'])

                # Check before flying that compass is calibrated
                sinYaw = sin(radians( vehicle.attitude['heading'] ))
                cosYaw = cos(radians( vehicle.attitude['heading'] ))

                #desiredRoll  = utils.toPWM(math.degrees( (rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / utils.g) ))
                #desiredPitch = utils.toPWM(math.degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / utils.g) ))

                # Mellinger paper
                desiredRoll  = toPWM(degrees( (rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / utils.g) ),1)
                desiredPitch = toPWM(degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / utils.g) ),1)

                # Murray 
                #desiredRoll  = utils.toPWM(math.degrees( (rPIDvalue * cosYaw - pPIDvalue * sinYaw) * (1 / utils.g) ),1)
                #desiredPitch = utils.toPWM(math.degrees( (pPIDvalue * cosYaw - rPIDvalue * sinYaw) * (1 / utils.g) ),1)

                # Check automatic mode on joystick and limit roll and pitch action
                if udp.message[7] == 1:
                    rcCMD[0] = limit(desiredRoll,1300,1700)
                    rcCMD[1] = limit(desiredPitch,1300,1700)

                # Make sure commands are inside the limits
                rcCMD = [limit(n,1000,2000) for n in rcCMD]

                # Send commands to the multiwii and update vehicle attitude (test!!!)
                vehicle.sendCMDreceiveATT(16,MultiWii.SET_RAW_RC,rcCMD)
            else:
                # Nothing to do but reset the rcCMD and the integrators (perhaps...)
                rcCMD = [1500,1500,1500,1000]
                #rollPID.resetIntegrator()
                #pitchPID.resetIntegrator()
                #rollPID.reset_I()
                #pitchPID.reset_I()
    except Exception,error:
        print "Error in control thread: "+str(error)

# Function to manage data, print it and save it in a log 
def logit():
    try:
        st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
        f = open("logs/mw-hc-"+st, "w")
        logger = csv.writer(f)
        logger.writerow(('timestamp','elapsed','angx','angy','heading','x','y','z','roll','pitch','yaw','throttle','rPID','pPID'))
        while True:
            if udp.active:
                # Save log
                logger.writerow((vehicle.attitude['timestamp'],vehicle.attitude['elapsed'], \
                                 vehicle.attitude['angx'],vehicle.attitude['angy'],vehicle.attitude['heading'], \
                                 currentPos['x'],currentPos['y'],currentPos['z'], \
                                 rcCMD[0],rcCMD[1],rcCMD[2],rcCMD[3], \
                                 rPIDvalue,pPIDvalue ))
                time.sleep(0.0125) # To record data at 80hz exactly
    except Exception,error:
        print "Error in logit thread: "+str(error)
        f.close()

if __name__ == "__main__":
    try:
        controlThread = threading.Thread(target=control)
        controlThread.daemon=True
        controlThread.start()
        logThread = threading.Thread(target=logit)
        logThread.daemon=True
        logThread.start()
        udp.startTwisted()
    except Exception,error:
        print "Error in main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()