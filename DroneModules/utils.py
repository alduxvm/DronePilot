#!/usr/bin/env python

"""utils.py: Several util functions needed in the DronePilot ecosystem."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

g = 9.81 # m/s2 - gravity

""" Function to map a value to another """
def toPWM(value):
    iMin = -50
    iMax = 50
    oMin = 1000
    oMax = 2000
    return round((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin, 0)

""" Function to map a value to another """
def mapping(value,iMin,iMax,oMin,oMax):
    return round((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin, 0)

""" Function to limit a value to specific range """
def limit(n, minn, maxn):
    return max(min(maxn, n), minn)