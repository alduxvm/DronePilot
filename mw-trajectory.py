#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-trajectory.py: Script that calculates pitch and roll movements for a vehicle 
    with MultiWii flight controller and a MoCap system in order to track a particular trajectory."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, datetime, csv, threading
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
ky = 500 / pi # Yaw controller gain

# Trajectory configuration
trajectory = 'circle'
w = (2*pi)/12 # It will take 6 seconds to complete a circle
# For Circle
radius = 0.8 # Circle radius
# Infinity trajectory configuration
a = 1.0
b = 1.2

# MRUAV initialization
vehicle = MultiWii("/dev/ttyUSB0")

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':1.0} # Set at the beginning, later is updated with the trajectory
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP from the motion capture system

# Velocity
velocities = {'x':0.0, 'y':0.0, 'z':0.0, 'fx':0.0, 'fy':0.0, 'fz':0.0}

# Initialize RC commands and pitch/roll to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000]
desiredRoll = desiredPitch = desiredYaw = 1500
desiredThrottle = 1000

# Controller PID's gains (Gains are considered the same for pitch and roll)
p_gains = {'kp': 2.61, 'ki':0.57, 'kd':3.41, 'iMax':2, 'filter_bandwidth':50} # Position Controller gains
h_gains = {'kp': 4.64, 'ki':1.37, 'kd':4.55, 'iMax':2, 'filter_bandwidth':50} # Height Controller gains
y_gains = {'kp': 1.0,  'ki':0.0,  'kd':0.0,  'iMax':2, 'filter_bandwidth':50} # Yaw Controller gains

# PID modules initialization
rollPID =   PID(p_gains['kp'], p_gains['ki'], p_gains['kd'], p_gains['filter_bandwidth'], 0, 0, update_rate, p_gains['iMax'], -p_gains['iMax'])
pitchPID =  PID(p_gains['kp'], p_gains['ki'], p_gains['kd'], p_gains['filter_bandwidth'], 0, 0, update_rate, p_gains['iMax'], -p_gains['iMax'])
heightPID = PID(h_gains['kp'], h_gains['ki'], h_gains['kd'], h_gains['filter_bandwidth'], 0, 0, update_rate, h_gains['iMax'], -h_gains['iMax'])
yawPID =    PID(y_gains['kp'], y_gains['ki'], y_gains['kd'], y_gains['filter_bandwidth'], 0, 0, update_rate, y_gains['iMax'], -y_gains['iMax'])
rPIDvalue = pPIDvalue = yPIDvalue = hPIDvalue = 0.0

# Filters initialization
f_yaw   = low_pass(20,update_rate)
f_pitch = low_pass(20,update_rate)
f_roll  = low_pass(20,update_rate)
f_desx  = low_pass(20,update_rate)
f_desy  = low_pass(20,update_rate)

# Calculate velocities
vel_x = velocity(20,update_rate)
vel_y = velocity(20,update_rate)
vel_z = velocity(20,update_rate)

# Function to update commands and attitude to be called by a thread
def control():
    global vehicle, rcCMD
    global rollPID, pitchPID, heightPID, yawPID
    global desiredPos, currentPos, velocities
    global desiredRoll, desiredPitch, desiredThrottle
    global rPIDvalue, pPIDvalue, yPIDvalue
    global f_yaw, f_pitch, f_roll, f_desx, f_desy
    global vel_x, vel_y, vel_z

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
            f = open("logs/mw-trajectory-"+st, "w")
            logger = csv.writer(f)
            # V -> vehicle | P -> pilot (joystick) | D -> desired position 
            # M -> motion capture | C -> commanded controls | sl -> Second marker | Mode 
            logger.writerow(('timestamp','Vroll','Vpitch','Vyaw','Proll','Ppitch','Pyaw','Pthrottle', \
                             'x','y','z','Dx','Dy','Dz','Mroll','Mpitch','Myaw','Mode','Croll','Cpitch','Cyaw','Cthrottle', \
                             'slx','sly','slz','slr','slp','sly', \
                             'vel_x', 'vel_fx', 'vel_y', 'vel_fy', 'vel_z', 'vel_fz' ))
        while True:
            # Variable to time the loop
            current = time.time()
            elapsed = 0

            # Update joystick commands from UDP communication, order (roll, pitch, yaw, throttle)
            for channel in range(0, 4):
                rcCMD[channel] = int(udp.message[channel])

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
            currentPos['x'] = udp.message[5]
            currentPos['y'] = udp.message[6]
            currentPos['z'] = -udp.message[7]

            # Get velocities of the vehicle
            velocities['x'],velocities['fx'] = vel_x.get_velocity(currentPos['x'])
            velocities['y'],velocities['fy'] = vel_y.get_velocity(currentPos['y'])
            velocities['z'],velocities['fz'] = vel_z.get_velocity(currentPos['z'])

            # Update vehicle Attitude 
            vehicle.getData(MultiWii.ATTITUDE)

            # Desired position changed using joystick movements
            if udp.message[4] == 1:
                desiredPos['x'] = radius
                desiredPos['y'] = 0.0
                trajectory_step = 0.0
            if udp.message[4] == 2:
                if trajectory == 'circle':
                    desiredPos['x'], desiredPos['y'] = circle_trajectory(radius, w, trajectory_step)
                    trajectory_step += update_rate
                elif trajectory == 'infinity':
                    desiredPos['x'], desiredPos['y'] = infinity_trajectory(a, b, w, trajectory_step)
                    trajectory_step += update_rate

            # Filter new values before using them
            heading = f_yaw.update(udp.message[12])

            # PID updating, Roll is for Y and Pitch for X, Z is negative
            rPIDvalue = rollPID.update(desiredPos['y']   - currentPos['y'])
            pPIDvalue = pitchPID.update(desiredPos['x']  - currentPos['x'])
            hPIDvalue = heightPID.update(desiredPos['z'] - currentPos['z'])
            yPIDvalue = yawPID.update(0.0 - heading)
            
            # Heading must be in radians, MultiWii heading comes in degrees, optitrack in radians
            sinYaw = sin(heading)
            cosYaw = cos(heading)

            # Conversion from desired accelerations to desired angle commands
            desiredRoll  = toPWM(degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / g) ), 1, 50)
            desiredPitch = toPWM(degrees( (pPIDvalue * cosYaw - rPIDvalue * sinYaw) * (1 / g) ), 1, 50)
            desiredThrottle = ((hPIDvalue + g) * vehicle_weight) / (cos(f_pitch.update(radians(vehicle.attitude['angx'])))*cos(f_roll.update(radians(vehicle.attitude['angy']))))
            desiredThrottle = round((desiredThrottle / kt) + u0 ,0)
            desiredYaw = round(1500 - (yPIDvalue * ky) ,0)

            # Limit commands for safety
            if udp.message[4] == 1:
                rcCMD[0] = limit(desiredRoll,1200,1800)
                rcCMD[1] = limit(desiredPitch,1200,1800)
                rcCMD[2] = limit(desiredYaw,1000,2000)
                rcCMD[3] = limit(desiredThrottle,1000,2000)
                mode = 'Auto'
            elif udp.message[4] == 2:
                rcCMD[0] = limit(desiredRoll,1200,1800)
                rcCMD[1] = limit(desiredPitch,1200,1800)
                rcCMD[2] = limit(desiredYaw,1000,2000)
                rcCMD[3] = limit(desiredThrottle,1000,2000)
                mode = 'Hybrid'
            else:
                # Prevent integrators to increase if they are not in use
                rollPID.resetIntegrator()
                pitchPID.resetIntegrator()
                heightPID.resetIntegrator()
                yawPID.resetIntegrator()
                mode = 'Manual'
            rcCMD = [limit(n,1000,2000) for n in rcCMD]

            # Send commands to vehicle
            vehicle.sendCMD(8,MultiWii.SET_RAW_RC,rcCMD)

            row =   (time.time(), \
                    vehicle.attitude['angx'], vehicle.attitude['angy'], vehicle.attitude['heading'], \
                    udp.message[0], udp.message[1], udp.message[2], udp.message[3], \
                    currentPos['x'], currentPos['y'], currentPos['z'], desiredPos['x'], desiredPos['y'], desiredPos['z'], \
                    udp.message[11], udp.message[13], udp.message[12], \
                    udp.message[4], \
                    rcCMD[0], rcCMD[1], rcCMD[2], rcCMD[3], \
                    udp.message[8], udp.message[9], udp.message[10], udp.message[14],udp.message[15], udp.message[16], \
                    velocities['x'], velocities['fx'], velocities['y'], velocities['fy'], velocities['z'], velocities['fz'] )

            if logging:
                logger.writerow(row)

            if mode == 'Auto' or 'Manual':
                print "Mode: %s | X: %0.2f | Y: %0.2f | Z: %0.2f" % (mode, currentPos['x'], currentPos['y'], currentPos['z'])
            elif mode == 'Hybrid':
                print "Mode: %s | Des_X: %0.2f | Des_Y: %0.2f" % (mode, desiredPos['x'], desiredPos['y'])                

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