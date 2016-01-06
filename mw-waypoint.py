#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-waypoint.py: Script that sends commands to a MultiWii vehicle to follow 
    several waypoints inside the laboratory.
"""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Altax.net"

__license__ = "GPL"
__version__ = "1.0"
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
vehicle.getData(MultiWii.ATTITUDE)

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':0.5} # Set at the beginning (for now...)
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP

# Initialize RC commands and pitch/roll to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000]
desiredRoll = 1500
desiredPitch = 1500
desiredThrottle = 1000
mode = 'Manual'

# Controller PID's gains (Gains are considered the same for pitch and roll)
p_gains = {'kp': 2.61, 'ki':0.57, 'kd':3.41, 'iMax':2, 'filter_bandwidth':50} # Position Controller gains
h_gains = {'kp': 4.64, 'ki':1.37, 'kd':4.55, 'iMax':2, 'filter_bandwidth':50} # Height Controller gains

# PID modules initialization
rollPID =   PID(p_gains['kp'], p_gains['ki'], p_gains['kd'], p_gains['filter_bandwidth'], 0, 0, update_rate, p_gains['iMax'], -p_gains['iMax'])
rPIDvalue = 0.0
rollPID.setPoint(desiredPos['x'])
pitchPID =  PID(p_gains['kp'], p_gains['ki'], p_gains['kd'], p_gains['filter_bandwidth'], 0, 0, update_rate, p_gains['iMax'], -p_gains['iMax'])
pPIDvalue = 0.0
pitchPID.setPoint(desiredPos['y'])
heightPID = PID(h_gains['kp'], h_gains['ki'], h_gains['kd'], h_gains['filter_bandwidth'], 0, 0, update_rate, h_gains['iMax'], -h_gains['iMax'])
hPIDvalue = 0.0
heightPID.setPoint(desiredPos['z'])

# Filters initialization
f_yaw   = low_pass(20,update_rate)
f_pitch = low_pass(20,update_rate)
f_roll  = low_pass(20,update_rate)


# Function that calculates distance between current position and desired and informs if the target has been reach.
# Waypoint variable is of the form: wp = {'x':0.0, 'y':0.0, 'z':1.0}
def go_to(target):
    global desiredPos, currentPos
    global rollPID, pitchPID, heightPID

    timeout = 15
    min_distance = 0.05
    start = time.time()
    desiredPos = target

    # Update PID values
    rollPID.setPoint(desiredPos['x'])
    pitchPID.setPoint(desiredPos['y'])
    heightPID.setPoint(desiredPos['z'])

    while True:
        current = time.time() - start
        distance = sqrt(pow(desiredPos['x'] - currentPos['x'],2) + pow(target['y'] - currentPos['y'],2) + pow(target['z'] - currentPos['z'],2) )
        print " > %0.2f Travelling to WP, distance to target = %0.4f" % (current, distance)
        if distance <= min_distance:
            print " - Reached target location"
            break;
        if current  >= timeout:
            print " - Timeout to reach location"
            break;
        time.sleep(0.1)


# Function that will change the desired position according to coordinates defined inside.
def mission():
    while True:
        # Only start the mission if the vehicle mode is set to Auto, this check 
        if 'Auto' in mode:

            # Waypoint definition
            wp1 = {'x':0.0, 'y':0.0, 'z':1.0}
            wp2 = {'x':0.0, 'y':0.0, 'z':0.5}

            print "¡¡Starting automatic mission!!"
            time.sleep(1)

            print "Traveling to waypoint 1: ", wp1
            go_to(wp1)
            # Maintain current position for certain time
            time.sleep(5)

            print "Traveling to waypoint 2: ", wp1
            go_to(wp1)
            # Maintain current position for certain time
            time.sleep(5)
        else:
            print "\tMode: %s | Z: %0.3f | X: %0.3f | Y: %0.3f " % (mode, currentPos['z'], currentPos['x'], currentPos['y'])
            time.sleep(0.5)

# Function to handle the flight management to be called by a thread, it communicates with the vehicle,
# it computes PID corrections and logs data. Is a loop that works at a desired update rate.
def flight_management():
    global vehicle, rcCMD
    global rollPID, pitchPID, heightPID
    global desiredPos, currentPos
    global desiredRoll, desiredPitch, desiredThrottle, mode
    global rPIDvalue, pPIDvalue
    global f_yaw, f_pitch, f_roll

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
            # V -> vehicle | P -> pilot (joystick) | M -> motion capture | C -> commanded controls
            logger.writerow(('timestamp','Vroll','Vpitch','Vyaw','Proll','Ppitch','Pyaw','Pthrottle','x','y','z','Mroll','Mpitch','Myaw','Mode','Croll','Cpitch','Cyaw','Cthrottle'))
        while True:

            # Update joystick commands from UDP communication, order (roll, pitch, yaw, throttle)
            rcCMD[0] = udp.message[0]
            rcCMD[1] = udp.message[1]
            rcCMD[2] = udp.message[2]
            rcCMD[3] = udp.message[3]

            # Coordinate map from Optitrack in the MAST Lab: X, Y, Z. NED: If going up, Z is negative. 
            ######### WALL ########
            #Door      | x+       |
            #          |          |
            #          |       y+ |
            #---------------------|
            # y-       |          |
            #          |          |
            #        x-|          |
            #######################
            # Update current position of the vehicle
            currentPos['x'] = udp.message[4]
            currentPos['y'] = udp.message[5]
            currentPos['z'] = -udp.message[6]

            # Update Attitude 
            vehicle.getData(MultiWii.ATTITUDE)

            # Filter new values before using them
            heading = f_yaw.update(udp.message[9])

            # PID updating, Roll is for Y and Pitch for X, Z is negative
            rPIDvalue = rollPID.update(currentPos['y'])
            pPIDvalue = pitchPID.update(currentPos['x'])
            hPIDvalue = heightPID.update(currentPos['z'])
            
            # Heading must be in radians, MultiWii heading comes in degrees, optitrack in radians
            sinYaw = sin(heading)
            cosYaw = cos(heading)

            # Conversion from desired accelerations to desired angle commands
            desiredRoll  = toPWM(degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / g) ),1)
            desiredPitch = toPWM(degrees( (pPIDvalue * cosYaw - rPIDvalue * sinYaw) * (1 / g) ),1)
            desiredThrottle = ((hPIDvalue + g) * vehicle_weight) #/ (cos(f_pitch.update(radians(vehicle.attitude['angx'])))*cos(f_roll.update(radianas(vehicle.attitude['angy']))))
            desiredThrottle = (desiredThrottle / kt) + u0

            # Limit commands for safety
            if udp.message[7] == 1:
                rcCMD[0] = limit(desiredRoll,1000,2000)
                rcCMD[1] = limit(desiredPitch,1000,2000)
                rcCMD[3] = limit(desiredThrottle,1000,2000)
                mode = 'Auto'
            else:
                # Prevent integrators/derivators to increase if they are not in use
                rollPID.resetIntegrator()
                pitchPID.resetIntegrator()
                heightPID.resetIntegrator()
                mode = 'Manual'
            rcCMD = [limit(n,1000,2000) for n in rcCMD]

            # Send commands to vehicle
            vehicle.sendCMD(8,MultiWii.SET_RAW_RC,rcCMD)

            row =   (time.time(), \
                    vehicle.attitude['angx'], vehicle.attitude['angy'], vehicle.attitude['heading'], \
                    #vehicle.rawIMU['ax'], vehicle.rawIMU['ay'], vehicle.rawIMU['az'], vehicle.rawIMU['gx'], vehicle.rawIMU['gy'], vehicle.rawIMU['gz'], \
                    #vehicle.rcChannels['roll'], vehicle.rcChannels['pitch'], vehicle.rcChannels['throttle'], vehicle.rcChannels['yaw'], \
                    udp.message[0], udp.message[1], udp.message[2], udp.message[3], \
                    udp.message[4], udp.message[5], udp.message[6], \
                    udp.message[8], udp.message[9], udp.message[10], \
                    udp.message[7], \
                    rcCMD[0], rcCMD[1], rcCMD[2], rcCMD[3]) 
            if logging:
                logger.writerow(row)

            #print "\tMode: %s | Z: %0.3f | X: %0.3f | Y: %0.3f " % (mode, currentPos['z'], currentPos['x'], currentPos['y'])
            # Wait time (not ideal, but its working) 
            time.sleep(update_rate)  

    except Exception,error:
        print "Error in control thread: "+str(error)

if __name__ == "__main__":
    try:
        flight_management_thread = threading.Thread(target=flight_management)
        flight_management_thread.daemon=True
        flight_management_thread.start()
        mission_thread = threading.Thread(target=mission)
        mission_thread.daemon=True
        mission_thread.start()
        udp.startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()