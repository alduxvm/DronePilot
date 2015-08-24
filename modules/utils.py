#!/usr/bin/env python

"""utils.py: Several utilitarian functions needed in the DronePilot ecosystem."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2015 Aldux.net"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

g = 9.81 # m/s2 - gravity

""" Function to map a value to another """
def toPWM(value, option):
    iMin = -50
    iMax = 50
    if option == 1: # Normal
    	oMin = 1000
    	oMax = 2000
    elif option == -1: # Inverted
    	oMin = 2000
    	oMax = 1000
    return round((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin, 0)

""" Function to map a value to another """
def mapping(value,iMin,iMax,oMin,oMax):
    return round((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin, 0)

""" Function to limit a value to specific range """
def limit(n, minn, maxn):
    return max(min(maxn, n), minn)


class PID:
    def __init__(self,p,i,d):
        self.kP=p
        self.kI=i
        self.kD=d
        self.target=0

        self.lastError=0
        self.integrator=0

    def setTarget(self,newTarget):
        self.target=newTarget
        self.integrator=0

    def step(self,currentValue):
        """
        Calculates the error and derives a desired output value.
        """
        # determine the error by simply looking at the difference between
        # current value and target value.
        error=currentValue-self.target

        # Build the output by summing the contributions of the
        # proportional, integral, and derivative models.
        output= (self.kP * error
                 + self.kI * self.integrator
                 + self.kD * (error - self.lastError)
                 )

        # Remember the error for the derivative model
        self.lastError=error
        # Add the error to the integral model
        self.integrator+=error

        return output