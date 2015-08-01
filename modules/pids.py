#!/usr/bin/env python

"""pids.py: Discrete PID controller."""

__author__ = "Simon Levy"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"
__downloaded__ = "https://github.com/simondlevy/PyQuadSim/blob/master/pidcontrol.py"

import math

class PID_Controller(object):
    '''
    General PID control class. 
    '''

    def __init__(self, Kp, Ki, Kd):
        '''
        Constructs a new PID_Controller object.
        '''
        
        # Parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # State variables
        self.Eprev = 0
        self.Stdt = 0
        self.t = 0
        
    def getCorrection(self, target, actual, dt=1):
        '''
        Returns current PID correction based on target value and actual value.
        '''
              
        E = target - actual
    
        # dE / dt
        dEdt = (E - self.Eprev) / dt if self.t > 0 else 0
        
        if abs(dEdt) > 1:
            dEdt = 0
        
        # Integral E / dt
        self.Stdt += (E + self.Eprev)*dt if self.t > 0 else 0
    
        correction = self.Kp*E + self.Ki*self.Stdt + self.Kd*dEdt
    
        # Update
        self.t += 1
        self.Eprev = E
        
        return correction