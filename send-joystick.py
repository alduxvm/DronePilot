#!/usr/bin/env python

"""joystick-sender.py: Reads a joystick device using pygame and sends the information via UDP."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Altax.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import socket, struct, time
import pygame
from modules.utils import *

# Main configuration
# IP address of the vehicle (raspberry pi address) 
UDP_IP = "127.0.0.1"
UDP_PORT = 51001 # This port match the ones using on other scripts

update_rate = 0.01 # 100 hz loop cycle
# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    current = time.time()
    elapsed = 0

    # Be sure to always send the data as floats
    message = [1500.0, 1500.0, 1500.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    buf = struct.pack('>' + 'd' * len(message), *message)
    sock.sendto(buf, (UDP_IP, UDP_PORT))
    print message

    # Make this loop work at update_rate
    while elapsed < update_rate:
        elapsed = time.time() - current