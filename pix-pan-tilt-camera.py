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
from modules.vision import *

# Connection to the vehicle
# SITL via TCP
#vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
# SITL/vehicle via UDP (connection coming from mavproxy.py)
vehicle = connect('udp:127.0.0.1:14549', wait_ready=True)
# Direct UART communication to Pixhawk
#vehicle = connect('/dev/ttyAMA0', wait_ready=True)

update_rate = 0.02 # 50 hertz update rate (maximum for DroneKit)

# Camera stuff
resX = 640
resY = 480

color_tracker = ColorTracker('white',False,resX,resY)

def move_servos(pan,tilt):
    vehicle.channel_override = { "6" : pan, "7" : tilt }  
    vehicle.flush()


def follow_tracker():
    """
    Function to calculate servo movements from a CV algorithm working on a thread and sending to the pixhawk those commands.
    """
    try:
        while True:
            if True:#vehicle.armed:
                current = time.time()
                elapsed = 0
                # Print message
                print color_tracker.tracker
                if color_tracker.tracker['found']:
                    pan_pulse  = toPWM(color_tracker.tracker['serx'],1)
                    tilt_pulse = toPWM(color_tracker.tracker['sery'],-1)
                    #vehicle.move_servo(1,pan_pulse)
                    #vehicle.move_servo(2,tilt_pulse)
                    move_servos(pan_pulse,tilt_pulse)
                else:
                    move_servos(1500,1500)

                # 100hz loop
                while elapsed < update_rate:
                    elapsed = time.time() - current
                # End of the main loop
            else:
                print "Waiting for vehicle to be armed to save data..."
                time.sleep(0.5)
    except Exception,error:
        print "Error in follow_tracker thread: "+str(error)


""" Section that starts the script """
try:
    print "\n\n"
    followThread = threading.Thread(target=follow_tracker)
    followThread.daemon=True
    followThread.start()
    visionThread = threading.Thread(target=color_tracker.findColor)
    visionThread.daemon=True
    visionThread.start()
    visionThread.join()
    udp.startTwisted()
except Exception,error:
    print "Error in main threads: "+str(error)
