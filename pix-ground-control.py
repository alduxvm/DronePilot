#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-ground-control.py -> Script that controls in a semi-autonomous way a vehicle equipped with a flight-stack. DroneKit 2.0 related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Aldux.net"

__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, threading, cmd, curses, sys, os
from dronekit import connect, VehicleMode, LocationGlobalRelative
from math import sin,cos,radians

# To prevent users exit using ctrl + c
#import signal,sys
#def catch_ctrl_C(sig,frame):
#    print "If you want to exit, type it."
#signal.signal(signal.SIGINT, catch_ctrl_C)

from modules.utils import *
from modules.pixVehicle import *

update_rate = 0.02 # 50 hz loop cycle


# Connection to the vehicle
# SITL via TCP
#vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
# SITL/vehicle via UDP (connection coming from mavproxy.py)
vehicle = connect('udp:127.0.0.1:14549', wait_ready=True)
# Direct UART communication to Pixhawk
#vehicle = connect('/dev/ttyAMA0', wait_ready=True)


def control_vehicle():
    try:
        while True:
            # Variable to time the loop
            current = time.time()
            elapsed = 0

            #print "Thread control"

            # Check vehicle battery
            if vehicle.battery.level < 5 and vehicle.mode.name!="LAND":
                print " Warning: [",vehicle.battery.level,"%] Battery level critical! Landing!"
                vehicle.mode = VehicleMode("LAND")

            # Wait until the update_rate is completed 
            time.sleep(update_rate)
            #while elapsed < update_rate:
            #    elapsed = time.time() - current
    except Exception,error:
        print "Error on on control thread: "+str(error)
    except KeyboardInterrupt:
        print "Keyboard Interrupt on control thread, exiting."
        pass

class user_interaction(cmd.Cmd):

    def __init__(self):
        cmd.Cmd.__init__(self)
        self.prompt = "Ready> "
        self.intro  = "\nWelcome to AltaX Ground Station console!\n"  ## defaults to None

    ###############################################
    #        Vehicle Command definitions          #
    ###############################################

    def do_status(self, option):
        """status [option]
        Prints the status of the vehicle.
        If 'all' is used as option, every parameter will be showed.
        """
        if option:
            print "Autopilot Firmware version: %s" % vehicle.version
            print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
            print "Global Location: %s" % vehicle.location.global_frame
            print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
            print "Local Location: %s" % vehicle.location.local_frame    #NED
            print "Attitude: %s" % vehicle.attitude
            print "Velocity: %s" % vehicle.velocity
            print "GPS: %s" % vehicle.gps_0
            print "Groundspeed: %s" % vehicle.groundspeed
            print "Airspeed: %s" % vehicle.airspeed
            print "Gimbal status: %s" % vehicle.gimbal
            print "Battery: %s" % vehicle.battery
            print "EKF OK?: %s" % vehicle.ekf_ok
            print "Last Heartbeat: %s" % vehicle.last_heartbeat
            print "Rangefinder: %s" % vehicle.rangefinder
            print "Rangefinder distance: %s" % vehicle.rangefinder.distance
            print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
            print "Heading: %s" % vehicle.heading
            print "Is Armable?: %s" % vehicle.is_armable
            print "System status: %s" % vehicle.system_status.state
            print "Mode: %s" % vehicle.mode.name    # settable
            print "Armed: %s" % vehicle.armed    # settable
        else:
            print "\nMod:%s %s | Pwr:%s%%, %0.1fv, %0.1fa | Alt:%0.1fm | %s\n" % (vehicle.mode.name[:7], vehicle.armed, vehicle.battery.level, float(vehicle.battery.voltage), float(vehicle.battery.current), float(vehicle.location.global_relative_frame.alt), vehicle.gps_0 )

    def do_param(self, line):
        """param 
        Prints ALL of the parameters of the vehicle.
        """
        for key, value in vehicle.parameters.iteritems():
            print "%20s %20s" % (key,value)

    def do_mode(self, mode):
        """mode [desired_mode]
        Changes the mode of the vehicle to the selected one.
        Default mode is guided.
        Modes: ['STABILIZE','ALT_HOLD','LAND','AUTO','GUIDED','LOITER','RTL','CIRCLE']
        """
        if mode:
            modes = ['STABILIZE','ALT_HOLD','LAND','AUTO','GUIDED','LOITER','RTL','CIRCLE']
            # If mode is Circle, the altitude will fall for SITL, an RC override will prevent this.
            if mode in ['circle']:
                vehicle.channels.overrides['3'] = 1500 # RC Override to ensure it will hold altitude.
            if mode.upper() in modes:
                vehicle.mode = VehicleMode(mode.upper())
            else:
                print "Invalid flight mode"
        else:
            vehicle.mode = VehicleMode("GUIDED")

    def do_takeoff(self, altitude):
        """takeoff [altitude]
        Attempts to takeoff the vehicle at the specified altitude.
        If altitude not defined, the default it 10 meters.
        """
        if not vehicle.armed:
            if altitude:
                print "\n\tAttempting to start take off!!"
                print "\tTaking off at %s meters\n" % altitude
                arm_and_takeoff(vehicle, float(altitude))
            else:
                print "\n\tAttempting to start take off!!"
                print "\tTaking off at default 10 meters\n"
                arm_and_takeoff(vehicle, 10)
        else:
            print "\n\tVehicle already flying.\n"
            pass

    def do_land(self, line):
        """land
        Lands the vehicle in the current location.
        Be careful as it can land on water!
        """
        print "\n\tLanding!\n"
        vehicle.mode = VehicleMode("LAND")
        try:
            while vehicle.armed:
                print " -> Alt:", vehicle.location.global_relative_frame.alt
                time.sleep(0.5)
        except KeyboardInterrupt:
            print "KeyInterrupt on LAND."
            pass

    def do_rtl(self, line):
        """rtl
        Return to launch to the home location.
        """
        print "\n\tReturning to launch and landing!\n"
        vehicle.mode = VehicleMode("RTL")
        try:
            while vehicle.armed:
                print " -> Alt: ", vehicle.location.global_relative_frame.alt
                time.sleep(0.5)
        except KeyboardInterrupt:
            print "KeyInterrupt on RTL."
            pass

    def do_position(self, line):
        """position
        Manual repositioning of the vehicle using the arrows from a keyboard.
        """
        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        screen.keypad(True)
        screen.addstr(0, 4,"Hit 'q' to exit this mode")
        screen.addstr(3, 15, '^') 
        screen.addstr(4, 17, '>')
        screen.addstr(4, 13, '<') 
        screen.addstr(4, 10, 'L')
        screen.addstr(4, 20, 'R')
        screen.addstr(5, 15, 'v')
        screen.addstr(7, 13, 'Brake')
        screen.addstr(0, 0, '')
        text="Pwr:%s%%, %0.1fv, %0.1fa | Alt:%0.1fm \n Vel:%0.1fx, %0.1fy, %0.1fz" % (vehicle.battery.level, float(vehicle.battery.voltage), float(vehicle.battery.current), float(vehicle.location.global_relative_frame.alt), float(vehicle.velocity[0]), float(vehicle.velocity[1]), float(vehicle.velocity[2]))
        screen.addstr(10, 1, text)
        screen.addstr(0, 0, '')
        #current = time.time()
        #elapsed = 0
        try:
            while True:
                char = screen.getch()
                text="Pwr:%s%%, %0.1fv, %0.1fa | Alt:%0.1fm \n Vel:%0.1fx, %0.1fy, %0.1fz" % (vehicle.battery.level, float(vehicle.battery.voltage), float(vehicle.battery.current), float(vehicle.location.global_relative_frame.alt), float(vehicle.velocity[0]), float(vehicle.velocity[1]), float(vehicle.velocity[2]))
                f=1*cos(radians(vehicle.heading))
                b=1*sin(radians(vehicle.heading))
                l=1*cos(radians(vehicle.heading-90))
                r=1*sin(radians(vehicle.heading-90))
                if char == ord('q'):
                    break
                elif char == curses.KEY_RIGHT:
                    screen.addstr(10, 1, text)
                    screen.addstr(0, 0, '')
                    #goto_position_target_local_ned(vehicle,0,-1,-10)
                    #time.sleep(1)
                    send_ned_velocity(vehicle,-l,-r,0,0.02)
                    #send_ned_velocity(vehicle,0,0,0,0.01)
                elif char == curses.KEY_LEFT:
                    screen.addstr(10, 1, text)
                    screen.addstr(0, 0, '')
                    #goto_position_target_local_ned(vehicle,0,1,-10)
                    #time.sleep(1)  
                    send_ned_velocity(vehicle,l,r,0,0.02) 
                    #send_ned_velocity(vehicle,0,0,0,0.01)
                elif char == curses.KEY_UP:
                    screen.addstr(10, 1, text)
                    screen.addstr(0, 0, '')     
                    #goto_position_target_local_ned(vehicle,1,0,-10)
                    #time.sleep(1) 
                    send_ned_velocity(vehicle,f,b,0,0.02)
                    #send_ned_velocity(vehicle,0,0,0,0.01)
                elif char == curses.KEY_DOWN:
                    screen.addstr(10, 1, text)
                    screen.addstr(0, 0, '')
                    #goto_position_target_local_ned(vehicle,-1,0,-10)
                    #time.sleep(1)
                    send_ned_velocity(vehicle,-f,-b,0,0.02)
                    #send_ned_velocity(vehicle,0,0,0,0.01)
                elif char == ord('a'):
                    screen.addstr(10, 1, text)
                    screen.addstr(0, 0, '')     
                    #goto_position_target_local_ned(vehicle,1,0,-10)
                    #time.sleep(1) 
                    condition_yaw(vehicle,vehicle.heading-5)
                    #send_ned_velocity(vehicle,0,0,0,0.01)
                elif char == ord('s'):
                    screen.addstr(10, 1, text)
                    screen.addstr(0, 0, '')     
                    #goto_position_target_local_ned(vehicle,1,0,-10)
                    #time.sleep(1) 
                    condition_yaw(vehicle,vehicle.heading+5)   
                    #send_ned_velocity(vehicle,0,0,0,0.01)
                elif char == ord('b'):
                    screen.addstr(10, 1, text)
                    screen.addstr(0, 0, '')     
                    send_ned_velocity(vehicle,0,0,0,1)
                elif char == ord('r'):
                    screen.addstr(10, 1, text)
                    screen.addstr(0, 0, '')     
                #elapsed = time.time() - current
                #screen.addstr(2,1,str(round(elapsed,2)))
        finally:
            curses.nocbreak(); screen.keypad(0); curses.echo()
            curses.endwin()
            sys.stdout = os.fdopen(0, 'w', 0)

    def do_altitude(self, altitude):
        """altitude [how high?]
        Adjust vehicle altitude
        """
        if altitude and altitude > 0.0 and vehicle.armed:
            print "\n\tChanging vehicle altitude %0.1f meters!\n" % float(altitude)
            vehicle.mode = VehicleMode("GUIDED")
            location = LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, float(altitude))
            try:
                go_to_alt(vehicle, location)
            except KeyboardInterrupt:
                print "KeyInterrupt on Altitude change."
                pass
        else:
            print "\n\tInvalid altitude or vehicle not flying.\n"
            pass

    ###############################################
    # Default command definitions for the console #
    ###############################################
    def emptyline(self):    
        """Do nothing on empty input line"""
        pass
    def do_exit(self, args):
        """Exits from the console"""
        return -1
    def do_q(self, args):
        """Exits from the console"""
        return -1
    def postloop(self):
        """Take care of any unfinished business.
        """
        cmd.Cmd.postloop(self)   ## Clean up command completion
        print "Exiting..."

if __name__ == "__main__":
    try:
        control_Thread = threading.Thread(target=control_vehicle)
        control_Thread.daemon=True
        #control_Thread.start()

        try: 
            user_interaction().cmdloop()
        except Exception,error:
            print "Error on console: "+str(error)

    except Exception,error:
        print "Error on main: "+str(error)
    except KeyboardInterrupt:
        print "Keyboard Interrupt on main, exiting."
        exit()