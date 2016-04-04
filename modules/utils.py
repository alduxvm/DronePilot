#!/usr/bin/env python

"""utils.py: Several utilitarian functions needed in the DronePilot ecosystem."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Aldux.net"

__license__ = "GPL"
__version__ = "1.1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time

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

""" Discrete PID control Class """
class PID:
    def __init__(self, P, I, D, filter_bandwidth, Derivator=0, Integrator=0, dt=0.01, Integrator_max=1.0, Integrator_min=-1.0):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.dt=dt
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min
        self.set_point=0.0
        self.error=0.0
        self.filter_bandwidth=filter_bandwidth
        self.filter=0.0
        self.filter_past=0.0

    def update(self, error):
        """
        Calculate PID output value for given reference input and feedback
        """

        #self.error = self.set_point - current_value
        self.error = error

        # Proportional term
        self.P_value = self.Kp * self.error

        # Filter
        self.filter = self.filter_past + self.dt * ( self.filter_bandwidth * ( self.error - self.filter_past ) )
        self.filter_past = self.filter

        # Derivative term
        self.D_value = self.Kd * (( self.filter - self.Derivator ) / self.dt )
        self.Derivator = self.filter

        # Integral term
        self.Integrator = self.Integrator + self.error * self.dt

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        #self.Integrator=0
        #self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

    def resetIntegrator(self):
        self.Integrator=0


""" Low Pass Filter """
class low_pass:
    def __init__(self,bandwidth,dt):
        self.filter_bandwidth=bandwidth
        self.dt=dt
        self.filter=0.0
        self.filter_past=0.0
    def update(self,current_value):
        self.filter = self.filter_past + self.dt * ( self.filter_bandwidth * ( current_value - self.filter_past ) )
        self.filter_past = self.filter
        return self.filter