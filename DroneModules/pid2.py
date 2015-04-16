#!/usr/bin/python
"""
pid class : implements a pid controller

"""

import math
import time

class pid(object):

    def __init__(self, initial_p=0, initial_i=0, initial_d=0, initial_imax=0):
        # default config file
        self.p_gain = initial_p
        self.i_gain = initial_i
        self.d_gain = initial_d
        self.imax = abs(initial_imax)
        self.integrator = 0
        self.last_error = None
        self.last_update = time.time() 

    # __str__ - print position vector as string
    def __str__(self):
        return "P:%s,I:%s,D:%s,IMAX:%s,Integrator:%s" % (self.p_gain, self.i_gain, self.d_gain, self.imax, self.integrator)

    # get_dt - returns time difference since last get_dt call
    def get_dt(self, max_dt):
        now = time.time()
        time_diff = now - self.last_update
        self.last_update = now
        if time_diff > max_dt:
            return 0.0
        else:
            return time_diff

    # get_p - return p term
    def get_p(self, error):
        return self.p_gain * error

    # get_i - return i term
    def get_i(self, error, dt):
        self.integrator = self.integrator + error * self.i_gain * dt
        self.integrator = min(self.integrator, self.imax)
        self.integrator = max(self.integrator, -self.imax)
        return self.integrator

    # get_d - return d term
    def get_d(self, error, dt):
        if self.last_error is None:
            self.last_error = error
        ret = (error - self.last_error) * self.d_gain * dt
        self.last_error = error
        return ret

    # get pi - return p and i terms
    def get_pi(self, error, dt):
        return self.get_p(error) + self.get_i(error,dt)

    # get pid - return p, i and d terms
    def get_pid(self, error, dt):
        return self.get_p(error) + self.get_i(error,dt) + self.get_d(error, dt)

    # get_integrator - return built up i term
    def get_integrator(self):
        return self.integrator

    # reset_I - clears I term
    def reset_I(self):
        self.integrator = 0

    # main - used to test the class
    def main(self):

        # print object
        print "Test PID: %s" % test_pid

        # run it through a test
        for i in range (0, 100):
            result_p = test_pid.get_p(i)
            result_i = test_pid.get_i(i, 0.1)
            result_d = test_pid.get_d(i, 0.1)
            result = result_p + result_i + result_d
            print "Err %s, Result: %f (P:%f, I:%f, D:%f, Int:%f)" % (i, result, result_p, result_i, result_d, self.get_integrator())

# run the main routine if this is file is called from the command line
if __name__ == "__main__":
    # create pid object P, I, D, IMAX
    test_pid = pid(1.0, 0.5, 0.01, 50)
    test_pid.main()