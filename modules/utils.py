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

""" Kalman Filter Class """
class KalmanFilter(object):
    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate