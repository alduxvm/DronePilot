#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-height-controller.py: Script that calculates throttle command for a vehicle 
    with MultiWii flight controller and a HC-SR04 sonar in order to keep a specified altitude."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

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

# Main configuration
logging = True
update_rate = 0.01 # 100 hz loop cycle
vehicle_weight = 0.84 # Kg
u0 = 1000 # Zero throttle command
uh = 1360 # Hover throttle command
kt = vehicle_weight * g / (uh-u0)

# MRUAV initialization
vehicle = MultiWii("/dev/ttyUSB0")

# Position coordinates [x, y, x] 
desiredPos = {'z':1.0} # Set at the beginning
currentPos = {'z':0.0} # It will be updated using readings from the HC-SR04 sonar

# Initialize RC commands and pitch/roll to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000]
desiredThrottle = 1000

# Controller PID's gains (Gains are considered the same for pitch and roll)
h_gains = {'kp': 4.64, 'ki':1.37, 'kd':4.55, 'iMax':2, 'filter_bandwidth':50} # Height Controller gains

# PID modules initialization
heightPID = PID(h_gains['kp'], h_gains['ki'], h_gains['kd'], h_gains['filter_bandwidth'], 0, 0, update_rate, h_gains['iMax'], -h_gains['iMax'])
hPIDvalue = 0.0

# Filters initialization
f_height = low_pass(20,update_rate)
f_pitch  = low_pass(20,update_rate)
f_roll   = low_pass(20,update_rate)

# Function to update commands and attitude to be called by a thread
def control():
    global vehicle, rcCMD
    global heightPID
    global desiredPos, currentPos
    global desiredThrottle
    global hPIDvalue
    global f_height, f_pitch, f_roll

    while True:
        if udp.active:
            print "UDP server is active..."
            break
        else:
            print "Waiting for UDP server to be active..."
        time.sleep(0.5)

    try:
        if logging:
            st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
            f = open("logs/mw-"+st, "w")
            logger = csv.writer(f)
            # V -> vehicle | P -> pilot (joystick) | D -> desired position | M -> motion capture | C -> commanded controls
            logger.writerow(('timestamp','Vroll','Vpitch','Vyaw','Proll','Ppitch','Pyaw','Pthrottle', \
                             'z','Croll','Cpitch','Cyaw','Cthrottle'))
        while True:
            # Variable to time the loop
            current = time.time()
            elapsed = 0

            # Update joystick commands from UDP communication, order (roll, pitch, yaw, throttle)
            rcCMD[0] = udp.message[0]
            rcCMD[1] = udp.message[1]
            rcCMD[2] = udp.message[2]
            rcCMD[3] = udp.message[3]

            # Update current position of the vehicle
            #currentPos['z'] = sonarReading

            # Update Attitude 
            vehicle.getData(MultiWii.ATTITUDE)

            # Filter new values before using them
            currentPos['z'] = -udp.message[7]
            #height = f_height.update(currentPos['z'])

            # PID updating, Roll is for Y and Pitch for X, Z is negative
            hPIDvalue = heightPID.update(desiredPos['z'] - currentPos['z'])

            # Conversion from desired accelerations to desired angle commands
            desiredThrottle = ((hPIDvalue + g) * vehicle_weight) / (cos(f_pitch.update(radians(vehicle.attitude['angx'])))*cos(f_roll.update(radians(vehicle.attitude['angy']))))
            desiredThrottle = (desiredThrottle / kt) + u0

            # Limit commands for safety
            if udp.message[4] == 1:
                #rcCMD[0] = limit(desiredRoll,1000,2000)
                #rcCMD[1] = limit(desiredPitch,1000,2000)
                #rcCMD[2] = limit(desiredYaw,1000,2000)
                rcCMD[3] = limit(desiredThrottle,1000,2000)
                mode = 'Auto'
            else:
                # Prevent integrators/derivators to increase if they are not in use
                heightPID.resetIntegrator()
                mode = 'Manual'
            rcCMD = [limit(n,1000,2000) for n in rcCMD]

            # Send commands to vehicle
            vehicle.sendCMD(8,MultiWii.SET_RAW_RC,rcCMD)

            row =   (time.time(), \
                    vehicle.attitude['angx'], vehicle.attitude['angy'], vehicle.attitude['heading'], \
                    udp.message[0], udp.message[1], udp.message[2], udp.message[3], \
                    currentPos['z'], \
                    rcCMD[0], rcCMD[1], rcCMD[2], rcCMD[3]) 
            if logging:
                logger.writerow(row)

            print "Mode: %s | Z: %0.3f | FilterZ: %0.3f | Throttle: %d " % (mode, currentPos['z'], currentPos['z'], rcCMD[3])

            # Wait until the update_rate is completed 
            while elapsed < update_rate:
                elapsed = time.time() - current

    except Exception,error:
        print "Error in control thread: "+str(error)

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