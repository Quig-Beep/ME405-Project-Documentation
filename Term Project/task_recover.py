from task_share import Share
from micropython import const
from ulab import numpy as np
from sensor_class import sensor_array
import math
S0_WAIT = const(0)
S1_REVERSE = const(1)
S2_TURN = const(2)

class task_recover:
    def __init__(self,crash,recover_done,vL_r,vR_r,shareEstimator,pcStart,sL,sR,yaw,sens,recover_mot,follow):
        self._recover_mot : Share = recover_mot
        self._crash : Share = crash
        self._recover_done : Share = recover_done
        self._vL : Share = vL_r
        self._vR : Share = vR_r
        self._state = S0_WAIT
        self._shareEstimator : Share = shareEstimator
        self._pcStart : Share = pcStart
        self._sL : Share = sL
        self._sR : Share = sR
        self.reverse = 25
        self._targetL = 0
        self._targetR = 0
        self._k = 10
        self._yaw : Share = yaw
        self._yaw_des = 0
        self._kw = 100
        self._sens : sensor_array = sens
        self._follow : Share = follow
        self._count = 13
    def run(self):
        while True:
            if self._state == S0_WAIT:
                if self._crash.get():
                    self._crash.put(False)
                    self._follow.put(False)
                    self._vL.put(0)
                    self._vR.put(0)
                    self._shareEstimator.put(False)
                    self._pcStart.put(False)
                    self._targetL = self._sL.get() - self.reverse
                    self._targetR = self._sR.get() - self.reverse
                    self._recover_mot.put(True)
                    self._state = S1_REVERSE                   
            elif self._state == S1_REVERSE:
                vL = -100
                vR = -100
                self._vL.put(vL)
                self._vR.put(vR)
                self._count-=1

                if self._count == 0:
                    self._count = 13
                    self._state = S2_TURN

            elif self._state == S2_TURN:

                if self._sens.centroid() is not None:
                        self._vL.put(0)
                        self._vR.put(0)
                        self._recover_done.put(True)
                        self._recover_mot.put(False)
                        self._state = S0_WAIT
                else:
                    self._vL.put(-100)
                    self._vR.put(100)                           
            yield       

def wrap_angle(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi