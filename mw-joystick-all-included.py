#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" mw-joystick-test.py: Send joystick commands via UDP from a ground-station running Matlab to a FC running MultiWii software."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, threading
from modules.pyMultiwii import MultiWii


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



# RC commnads to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000,1000,1000,1000,1000]

# MRUAV initialization
#vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")
#vehicle = MultiWii("/dev/ttyUSB0")

# Function to update commands and attitude to be called by a thread
def updateCMDATT():
    global vehicle, rcCMD
    try:
        while True:
            if active:
                # Part for applying commands to the vehicle.
                rcCMD[0] = message[0]
                rcCMD[1] = message[1]
                rcCMD[2] = message[2]
                rcCMD[3] = message[3]
                #vehicle.sendCMDreceiveATT(16,MultiWii.SET_RAW_RC,rcCMD)
                #print vehicle.attitude
                print message
                time.sleep(0.01)
            else: 
                # Part for landing and disarming.
                #print modules.UDPserver.message
                print "no active!"
                time.sleep(1)
    except Exception,error:
        print "Error on updateCMDATT thread: "+str(error)
        updateCMDATT()

if __name__ == "__main__":
    try:
        vehicleThread = threading.Thread(target=updateCMDATT)
        vehicleThread.daemon=True
        vehicleThread.start()
        startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()