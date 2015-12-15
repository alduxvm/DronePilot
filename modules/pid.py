#!/usr/bin/env python

"""pid.py: Discrete PID controller."""

__author__ = "cnr437@gmail.com"

__license__ = "GPL"
__version__ = "1"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"
__downloaded__ = "http://code.activestate.com/recipes/577231-discrete-pid-controller/"

import time

class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P, I, D, Derivator=0, Integrator=0, dt=0.0125, Integrator_max=1.0, Integrator_min=-1.0):

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

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		# Proportional term
		self.P_value = self.Kp * self.error

		# Derivative term
		self.D_value = self.Kd * (( self.error - self.Derivator ) / self.dt )
		self.Derivator = self.error

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
		self.Integrator=0
		self.Derivator=0

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

if __name__ == "__main__":
    # create pid object P, I, D, IMAX
	p = PID(4.65, 1.13, 4.5, 0, 0, 10,-10)
	p.setPoint(50.0)
	for i in range (0, 100):
		start = time.time()
		pid = p.update(i)
		print "%d elapsed = %f error = %f" % (i,time.time()-start,pid)

