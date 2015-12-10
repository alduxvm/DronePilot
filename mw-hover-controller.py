#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-hover-controller.py: Script that calculates pitch and roll movements for a vehicle 
    with MultiWii flight controller and a MoCap system in order to keep a specified position.
"""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, datetime, csv, threading
from math import *
from modules.utils import *
from modules.pyMultiwii import MultiWii
import modules.UDPserver as udp
from modules.pid import PID

# MRUAV initialization
vehicle = MultiWii("/dev/ttyUSB0")

# Initialize vehicles attitude for further calculations
vehicle.getData(MultiWii.ATTITUDE)
heading = vehicle.attitude['heading']

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':0.0} # Set at the beginning (for now...)
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP

# Initialize RC commands and pitch/roll to be sent to the MultiWii 
#rcCMD = [1500,1500,1500,1000,1000,1000,1000,1000]
rcCMD = [1500,1500,1500,1000]
desiredRoll = 1500
desiredPitch = 1500
maxAngle = 1700
minAngle = 1300

# PID's initialization 
rgains = {'kp':0.5, 'ki':0.0, 'kd':0.8, 'iMax':1}
pgains = {'kp':0.5, 'ki':0.0, 'kd':0.8, 'iMax':1}
rPIDvalue = 0.0
pPIDvalue = 0.0

# PID module pids
rollPID =  PID(rgains['kp'], rgains['ki'], rgains['kd'], 0, 0, rgains['iMax'], -rgains['iMax'])
rollPID.setPoint(desiredPos['x'])
pitchPID = PID(pgains['kp'], pgains['ki'], pgains['kd'], 0, 0, pgains['iMax'], -pgains['iMax'])
pitchPID.setPoint(desiredPos['y'])

# Function to update commands and attitude to be called by a thread
def control():
    global vehicle, rcCMD
    global rollPID, pitchPID
    global desiredPos, currentPos
    global desiredRoll, desiredPitch
    global rPIDvalue, pPIDvalue

    while True:
        if udp.active:
            print "UDP server is active..."
            break
        else:
            print "Waiting for UDP server to be active..."
        time.sleep(0.5)

    try:
        #st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
        #f = open("logs/mw-"+st, "w")
        #logger = csv.writer(f)
        #logger.writerow(('timestamp','roll','pitch','yaw','proll','ppitch','throttle','pyaw','x','y','z'))
        while True:
            #elapsed = time.time()
            rcCMD[0] = udp.message[0]
            rcCMD[1] = udp.message[1]
            rcCMD[2] = udp.message[2]
            rcCMD[3] = udp.message[3]

            # Order of the position from Optitrack is: X, Z, Y
            currentPos['x'] = udp.message[5]
            currentPos['y'] = udp.message[4]
            currentPos['z'] = udp.message[6]

            # Update vehicle attitude
            #vehicle.getData(MultiWii.ATTITUDE)

            # Update PID's
            rPIDvalue = rollPID.update(currentPos['x'])
            pPIDvalue = pitchPID.update(currentPos['y'])

            # Calculate angles
            desiredRoll  = mapping(rPIDvalue, -rgains['iMax'], rgains['iMax'], minAngle, maxAngle )
            desiredPitch = mapping(pPIDvalue, -pgains['iMax'], pgains['iMax'], minAngle, maxAngle )

            # Limit commands for safety
            if udp.message[7] == 1:
                rcCMD[1] = limit(desiredRoll, minAngle , maxAngle)
                rcCMD[0] = limit(desiredPitch, minAngle, maxAngle)
            rcCMD = [limit(n,1000,2000) for n in rcCMD]

            # Send command to vehicle
            vehicle.sendCMD(16,MultiWii.SET_RAW_RC,rcCMD)

            # Print information for debugging purposes 
            print "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d" % (time.time(), \
                    vehicle.attitude['angx'], vehicle.attitude['angy'], udp.message[10], \
                    currentPos['x'], currentPos['y'], currentPos['z'], \
                    rPIDvalue, pPIDvalue, \
                    int(rcCMD[0]), int(rcCMD[1]), \
                    int(desiredRoll), int(desiredPitch) ) 

            # Save log
            #logger.writerow((time.time(), \
            #                 vehicle.attitude['angx'], vehicle.attitude['angy'], vehicle.attitude['heading'], \
                             #vehicle.rcChannels['roll'], vehicle.rcChannels['pitch'], vehicle.rcChannels['throttle'], vehicle.rcChannels['yaw'], \
            #                 udp.message[0], udp.message[1], udp.message[3], udp.message[2], \
            #                 udp.message[5], udp.message[4], udp.message[6] ))
            time.sleep(0.01) # 100hz 

    except Exception,error:
        print "Error in control thread: "+str(error)
        #f.close()

if __name__ == "__main__":
    try:
        logThread = threading.Thread(target=control)
        logThread.daemon=True
        logThread.start()
        udp.startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()

