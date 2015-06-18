#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" pix-logdata.py -> Script that logs data from a computer vision thread and from a flying multicopter with a Pixhawk. DroneApi related. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__maintainer__ = "Kyle Brown"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

# Dependancies
# * This script assumes that the pixhawk is connected to the raspeberry pi via the serial port (/dev/ttyAMA0)
# * In our setup, telemetry port 2 is configured at 115200 on the pixhawk
# * rpi camera connected (for computer vision)

# Usage:
# * mavproxy.py --master=/dev/ttyAMA0 --baudrate 115200 --aircraft testQuad
# * module load api
# * api start pix-logdata.py


from droneapi.lib import VehicleMode
from pymavlink import mavutil


import time
import numpy as np
import cv2

api = local_connect()
v = api.get_vehicles()[0]