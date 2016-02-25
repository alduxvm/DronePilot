#!/usr/bin/env python

"""UDPserver.py: Handles UDP twisted communications for reading a Optitrack Motion Capture System."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "1.2"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import struct, time, socket
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task

UDPport = 51001
message = [1500,1500,1500,1000,0,0,0,0,0,0,0,0,0,0,0,0]
active = False

"""
Current order for Aldux Simulink UDP script:
- UDP message size: 17

Joystick:
    Roll     -> 0
    Pitch    -> 1
    Yaw      -> 2
    Throttle -> 3
    Switch   -> 4

Trackable 1 (main):
    X -> 5    Att1 -> 11
    Y -> 6    Att2 -> 12
    Z -> 7    Att3 -> 13

Trackable 2 (extra):
    X -> 8    Att1 -> 14
    Y -> 9    Att2 -> 15
    Z -> 10   Att3 -> 16
"""

def timeout():
    global active, message
    if not active:
        # There is no UDP data, so give message "safe" commands
        message = [1500,1500,1500,1000,0,0,0,0,0,0,0,0,0,0,0,0]
    active = False

class twistedUDP(DatagramProtocol):

    def datagramReceived(self, data, (host, port)):
        global message, active
        active = True
        # In case of failure on receiving, check this:
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
    l.start(0.5) # Check for disconnection each 0.5 and send neutral commands
    reactor.listenUDP(UDPport, twistedUDP())
    reactor.run()