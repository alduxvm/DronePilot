#!/usr/bin/env python

"""send-joystick.py: Reads a joystick device using pygame and sends the information via UDP."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import socket, struct, time
import pygame
from modules.utils import *

# Main configuration
#UDP_IP = "127.0.0.1" # Localhost (for testing)
UDP_IP = "130.209.176.146" # Vehicle IP address
UDP_PORT = 51001 # This port match the ones using on other scripts

update_rate = 0.01 # 100 hz loop cycle
# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
except Exception,error:
    print "No joystick connected on the computer, "+str(error)

while True:
    current = time.time()
    elapsed = 0
    
    # Joystick reading
    pygame.event.pump()
    roll     = mapping(joystick.get_axis(0),-1.0,1.0,1000,2000)
    pitch    = mapping(joystick.get_axis(1),1.0,-1.0,1000,2000)
    yaw      = mapping(joystick.get_axis(2),-1.0,1.0,1000,2000)
    throttle = mapping(joystick.get_axis(3),1.0,-1.0,1000,2000)
    mode     = joystick.get_button(24)

    # Be sure to always send the data as floats
    # The extra zeros on the message are there in order for the other scripts to do not complain about missing information
    message = [roll, pitch, yaw, throttle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    buf = struct.pack('>' + 'd' * len(message), *message)
    sock.sendto(buf, (UDP_IP, UDP_PORT))
    
    print message

    # Make this loop work at update_rate
    while elapsed < update_rate:
        elapsed = time.time() - current