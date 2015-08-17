#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-joystick.py -> Script that send the vehicle joystick override using data from a UDP server. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, math
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil


""" ------------------- UDP module ------------------- """
""" Functions to be implemented inside a module - todo """


import struct, time, socket
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task

UDPport = 51001
message = [1500,1500,1500,1000,0,0,0,0]
active = False

def timeout():
    global active, message
    if not active:
        # There is no UDP data, so give message "safe" commands
        message = [1500,1500,1500,1000,0,0,0,0]
    active = False

class twistedUDP(DatagramProtocol):

    def datagramReceived(self, data, (host, port)):
        global message, active
        active = True
        #self.timeout.cancel()
        numOfValues = len(data) / 8
        mess=struct.unpack('>' + 'd' * numOfValues, data)
        message = [ round(element,6) for element in mess ]
        #print message
        #UDPmess.insert(0,time.time())
        #self.sendMSG(cfg.line,(cfg.UDPip, cfg.UDPportOut))
        #self.sendMSG("1",(cfg.UDPip, cfg.UDPportOut))
    #def sendMSG(self, data, (host, port)):
    #     self.transport.write(data, (host, port))

def startTwisted():
    l = task.LoopingCall(timeout)
    l.start(0.5) # Check for disconnection each 0.1 and send neutral commands
    reactor.listenUDP(UDPport, twistedUDP())
    reactor.run()

""" ----------------- end UDP module ------------------ """
""" --------------------------------------------------- """



api = local_connect()
vehicle = api.get_vehicles()[0]


""" Functions to be implemented inside a module - todo """

def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""
	print "Basic pre-arm checks"
	# Don't let the user try to fly autopilot is booting
	if vehicle.mode.name == "INITIALISING":
		print "Waiting for vehicle to initialise"
		time.sleep(1)
	while vehicle.gps_0.fix_type < 2:
		print "Waiting for GPS...:", vehicle.gps_0.fix_type
		time.sleep(1)
		
	print "Arming motors"
	# Copter should arm in GUIDED mode
	vehicle.mode    = VehicleMode("GUIDED")
	vehicle.armed   = True
	vehicle.flush()

	while not vehicle.armed and not api.exit:
		print " Waiting for arming..."
		time.sleep(1)

	print "Taking off!"
	vehicle.commands.takeoff(aTargetAltitude) # Take off to target altitude
	vehicle.flush()

	while not api.exit:
		print " Altitude: ", vehicle.location.alt
		if vehicle.location.alt>=aTargetAltitude*0.95: #Just below target, in case of undershoot.
			print "Reached target altitude"
			break;
		time.sleep(1)

def go_to(target):
	timeout = 20
	start = time.time()
	vehicle.commands.goto(target)
	vehicle.flush()
	
	while not api.exit:
			current = time.time() - start
			dTarget = math.sqrt(math.pow(target.lat-vehicle.location.lat,2)+math.pow(target.lon-vehicle.location.lon,2))
			print " ->%0.2f Traveling to WP, distance = %f" % (current, dTarget)
			if dTarget<=0.000005:
					print "Reached target location"
					break;
			if current >= timeout:
					print "Timeout to reach location"
					break;
			time.sleep(0.5)

def send_velocity_vector(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink.pde)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink.pde) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def condition_yaw(heading):
    msg = vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
            0,     # sequence
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
            2, # current - set to 2 to make it a guided command
            0, # auto continue
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0,          # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to update commands and attitude to be called by a thread
def joystick():
    try:
        while True:
            if active:
                # Part for applying commands to the vehicle.
                vehicle.channel_override = { "1" : message[0], "2" : message[1], \
                                             "3" : message[2], "4" : message[3] }
                vehicle.flush()
                #print message
                #time.sleep(0.01) # Maybe not needed?
    except Exception,error:
        print "Error on joystick thread: "+str(error)
        joystick()

""" Mission starts here """
if __name__ == "__main__":
    try:
        vehicleThread = threading.Thread(target=joystick)
        vehicleThread.daemon=True
        vehicleThread.start()
        startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()
