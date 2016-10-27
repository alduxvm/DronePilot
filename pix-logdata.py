#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-logdata.py -> Script that logs data from a vehicle and a MoCap system. DroneApi related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "2.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, datetime, csv, threading
from dronekit import connect, VehicleMode
import modules.UDPserver as udp
from modules.utils import *
from modules.pixVehicle import *

# Connection to the vehicle
# SITL via TCP
#vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
# SITL/vehicle via UDP (connection coming from mavproxy.py)
vehicle = connect('udp:127.0.0.1:14549', wait_ready=True)
# Direct UART communication to Pixhawk
#vehicle = connect('/dev/ttyAMA0', wait_ready=True)

update_rate = 0.02 # 50 hertz update rate (maximum for DroneKit)

def logit():
    """
    Function to manage data, print it and save it in a csv file, to be run in a thread
    """
    while True:
        if True:#udp.active:
            print "UDP server is active, starting..."
            break
        else:
            print "Waiting for UDP server to be active..."
        time.sleep(0.5)

    try:
        st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
        f = open("logs/"+"pix-"+st, "w")
        logger = csv.writer(f)
        logger.writerow(('timestamp','roll','pitch','yaw','vx','vy','vz','rc1','rc2','rc3','rc4','x','y','z'))
        while True:
            if vehicle.armed:
                current = time.time()
                elapsed = 0
                # Print message
                print "Roll = %0.4f Pitch = %0.4f Yaw= %0.4f" % (vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw)
                print "Vx = %0.4f Vy = %0.4f Vz= %0.4f" % (vehicle.velocity[0], vehicle.velocity[1], vehicle.velocity[2])
                print "RC1 = %d RC2 = %d RC3 = %d  RC4 = %d " % (vehicle.channels['1'], vehicle.channels['2'], vehicle.channels['3'], vehicle.channels['4'])
                #print "X = %0.2f Y = %0.2f Z = %0.2f" % (udp.message[5], udp.message[4], udp.message[6])
                # Save log
                row =   (current, \
                        vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw, \
                        vehicle.velocity[0], vehicle.velocity[1], vehicle.velocity[2], \
                        vehicle.channels['1'], vehicle.channels['2'], vehicle.channels['3'], vehicle.channels['4'], \
                        udp.message[5], udp.message[4], udp.message[6] )
                logger.writerow(row)
                # hz loop
                while elapsed < update_rate:
                    elapsed = time.time() - current
                # End of the main loop
            else:
                print "Waiting for vehicle to be armed to save data..."
                time.sleep(0.5)
    except Exception,error:
        print "Error in logit thread: "+str(error)
        f.close()


""" Section that starts the script """
try:
    print "\n\n"
    logThread = threading.Thread(target=logit)
    logThread.daemon=True
    logThread.start()
    udp.startTwisted()
except Exception,error:
    print "Error in main thread: "+str(error)
    vehicle.close
