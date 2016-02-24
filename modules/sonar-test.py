#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" sonar-test.py: Script that reads and calculates the distance using a HC-SR04 sonar. """

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
TRIG = 23 
ECHO = 24

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
update_rate = 0.1

def calculateDistance():
    GPIO.output(TRIG, False)
    time.sleep(update_rate)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = round(pulse_duration * 17150,2)
    return distance
    GPIO.cleanup()

while True:
    print "Distance = %0.2f" % calculateDistance()
    time.sleep(0.1)
