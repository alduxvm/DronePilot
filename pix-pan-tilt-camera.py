#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-logdata.py -> Script that logs data from a vehicle and a MoCap system. DroneApi related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1.5"
__maintainer__ = "Aldo Vargas"
__maintainer__ = "Kyle Brown"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, datetime, csv, threading
'''  To import own modules, you need to export the current path before importing the module.    '''
'''  This also means that mavproxy must be called inside the folder of the script to be called. ''' 
import os, sys
sys.path.append(os.getcwd())
import modules.UDPserver as udp
import modules.utils as utils
import modules.vision
import modules.pixVehicle

# Vehicle initialization
api = local_connect()
vehicle = api.get_vehicles()[0]

# Camera stuff
resX = 640
resY = 480

color_tracker = ColorTracker('white',False,resX,resY)

def logit():
    """
    Function to manage data, print it and save it in a csv file, to be run in a thread
    """
    while True:
        if udp.active:
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
            if True:#vehicle.armed:
                current = time.time()
                elapsed = 0
                # Print message
                if color_tracker.position['found']:
                    pan_pulse  = (color_tracker.position['x']-(resX/2))*Xpixelratio
                    tilt_pulse = (color_tracker.position['y']-(resY/2))*Ypixelratio
                    vehicle.move_servo(Pport,pan_pulse)
                    vehicle.move_servo(Tport,tilt_pulse)
                else:
                    vehicle.move_servo(Pport,1500)
                    vehicle.move_servo(Tport,1500)

                # 100hz loop
                while elapsed < 0.01:
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
    visionThread = threading.Thread(target=color_tracker.findColor)
    visionThread.daemon=True
    visionThread.start()
    #visionThread.join()
    udp.startTwisted()
except Exception,error:
    print "Error in main threads: "+str(error)
