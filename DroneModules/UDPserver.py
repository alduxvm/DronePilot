#!/usr/bin/env python

"""UDPserver.py: Handles UDP twisted communications for reading a Optitrack Motion Capture System."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2014 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import struct, time, socket
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task

UDPport = 51001
message = [1500,1500,1500,1000]
datagramRecieved = False

def timeout():
    global datagramRecieved, message
    if not datagramRecieved:
        #print "No UDP present"
        message = [1500,1500,1500,1000]
    datagramRecieved = False

class twistedUDP(DatagramProtocol):

    def datagramReceived(self, data, (host, port)):
        global message
        numOfValues = len(data) / 8
        mess=struct.unpack('>' + 'd' * numOfValues, data)
        message = [ round(element,4) for element in mess ]
        #UDPmess.insert(0,time.time())
        #self.sendMSG(cfg.line,(cfg.UDPip, cfg.UDPportOut))
        #self.sendMSG("1",(cfg.UDPip, cfg.UDPportOut))
    #def sendMSG(self, data, (host, port)):
    #     self.transport.write(data, (host, port))

def startTwisted():
    l = task.LoopingCall(timeout)
    l.start(1.0) # Check for disconnection each second and send neutral commands
    reactor.listenUDP(UDPport, twistedUDP())
    reactor.run()