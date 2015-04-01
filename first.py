#!/usr/bin/env python

"""first.py: TODO."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import serial, struct, time, threading
#from twisted.internet.protocol import DatagramProtocol
#from twisted.internet import reactor
from modules.pyMultiwii import MultiWii


if __name__ == "__main__":

    vehicle = MultiWii("/dev/tty.usbserial-AM016WP4")
    #vehicle = MultiWii("/dev/ttyUSB0")

    try:
        while True:
            vehicle.getData(MultiWii.ATTITUDE)
            
            # This is a dummy function, it needs to get done
            #vehicle.getPosition(UDP)
            
            # Calculate control using a PID (TODO)

            vehicle.sendCMD(16,MultiWii.SET_RAW_RC,data)

    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()