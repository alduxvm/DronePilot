#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-hover-controller2.py: Script that calculates pitch and roll movements for a vehicle 
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
from modules.pids import PID_Controller

# MRUAV initialization
vehicle = MultiWii("/dev/ttyUSB0")
vehicle.getData(MultiWii.ATTITUDE)
heading = vehicle.attitude['heading']

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':0.0} # Set at the beginning (for now...)
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP

# Initialize RC commands and pitch/roll to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000,1000,1000,1000,1000]
desiredRoll = 1500
desiredPitch = 1500
desiredRoll2 = 1500
desiredPitch2 = 1500

# PID's initialization 
rgains = {'kp':0.5, 'ki':0.0, 'kd':0.8, 'iMax':1}
pgains = {'kp':0.5, 'ki':0.0, 'kd':0.8, 'iMax':1}
rPIDvalue = 0.0
pPIDvalue = 0.0
rPIDvalue2 = 0.0
pPIDvalue2 = 0.0

# PID module pids
rollPID =  PID(rgains['kp'], rgains['ki'], rgains['kd'], 0, 0, rgains['iMax'], -rgains['iMax'])
rollPID.setPoint(desiredPos['x'])
pitchPID = PID(pgains['kp'], pgains['ki'], pgains['kd'], 0, 0, pgains['iMax'], -pgains['iMax'])
pitchPID.setPoint(desiredPos['y'])

rollPID2  = PID_Controller(rgains['kp'], rgains['ki'], rgains['kd'])
pitchPID2 = PID_Controller(pgains['kp'], pgains['ki'], pgains['kd'])

# Function to update commands and attitude to be called by a thread
def control():
    global vehicle, rcCMD
    global rollPID, pitchPID
    global rollPID2, pitchPID2
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
        process_variance = 1e-3
        estimated_measurement_variance = 16
        kalman_filter = KalmanFilter(process_variance, estimated_measurement_variance)
        kalman_yaw = 0
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

            # Update Attitude and heading
            vehicle.getData(MultiWii.ATTITUDE)

            # Kalman update
            kalman_filter.input_latest_noisy_measurement(vehicle.attitude['heading'])
            kalman_yaw = kalman_filter.get_latest_estimated_measurement()

            rPIDvalue = rollPID.update(currentPos['x'])
            pPIDvalue = pitchPID.update(currentPos['y'])

            rPIDvalue2 = rollPID2.getCorrection(desiredPos['x'],currentPos['x'])
            pPIDvalue2 = pitchPID2.getCorrection(desiredPos['y'],currentPos['y'])

            # Heading update
            heading = udp.message[8]
            sinYaw = sin(heading)
            cosYaw = cos(heading)

            # Mellinger paper
            desiredRoll  = toPWM(degrees( (rPIDvalue * sinYaw - pPIDvalue * cosYaw) * (1 / g) ),1)
            desiredPitch = toPWM(degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / g) ),1)
            desiredRoll2  = toPWM(degrees( (rPIDvalue2 * sinYaw - pPIDvalue2 * cosYaw) * (1 / g) ),1)
            desiredPitch2 = toPWM(degrees( (rPIDvalue2 * cosYaw + pPIDvalue2 * sinYaw) * (1 / g) ),1)

            #desiredRoll  = toPWM(degrees( (pPIDvalue * cosYaw - rPIDvalue * sinYaw) * (1 / g) ),1)
            #desiredPitch = toPWM(degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / g) ),1)
            #desiredRoll2  = toPWM(degrees( (pPIDvalue2 * cosYaw - rPIDvalue2 * sinYaw) * (1 / g) ),1)
            #desiredPitch2 = toPWM(degrees( (rPIDvalue2 * cosYaw + pPIDvalue2 * sinYaw) * (1 / g) ),1)

            # Limit commands for safety
            if udp.message[7] == 1:
                rcCMD[0] = limit(desiredRoll,1200,1800)
                rcCMD[1] = limit(desiredPitch,1200,1800)
            rcCMD = [limit(n,1000,2000) for n in rcCMD]

            # Send command to vehicle
            vehicle.sendCMD(16,MultiWii.SET_RAW_RC,rcCMD)

            #vehicle.sendCMD(16,MultiWii.SET_RAW_RC,rcCMD)
            #time.sleep(0.005) # Time to allow the Naze32 respond the last attitude command
            #vehicle.getData(MultiWii.ATTITUDE)
            #vehicle.getData(MultiWii.RC)
            #print "Time to ask two commands -> %0.3f" % (time.time()-elapsed)
            #print "%s %s" % (vehicle.attitude,rcCMD
            print "%.2f %.2f %.2f %.2f %.2f %d %d %d %d %d %d" % (time.time(), \
                vehicle.attitude['heading'], heading, \
                currentPos['x'], currentPos['y'], \
                int(rcCMD[0]), int(rcCMD[1]), \
                int(desiredRoll), int(desiredRoll), \
                int(desiredRoll2), int(desiredPitch2) ) 

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