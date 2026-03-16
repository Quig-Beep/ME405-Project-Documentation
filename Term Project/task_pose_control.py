from task_share import Share
from micropython import const
from ulab import numpy as np
import math

S0_WAIT = const(0)
S1_INIT_POSE = const(1)
S2_DRIVE = const(2)
S3_FINAL_POSE = const(3)

# ROMI Pose controller 

class pose_control:
    def __init__(self, xPos, yPos, estPsi, vL, vR, xDes, yDes, psiDes, pcStart):
        self._x : Share = xPos
        self._y : Share = yPos
        self._psi : Share = estPsi
        self._vL : Share = vL
        self._vR : Share = vR
        self._xDes : Share = xDes
        self._yDes : Share = yDes
        self._psiDes : Share = psiDes
        self._pcStart : Share = pcStart
        self._psi_init = 0
        self._kpsi = 650
        self._kpsi2 = 100
        self._kdist = 2.5
        self._x_des = 0
        self._y_des = 0
        self._psi_des = 0
        self._state = S0_WAIT
    def run(self):
        while(True):
            if not self._pcStart.get():
                self._state = S0_WAIT
            if self._state == S0_WAIT:
                self._vL.put(0)
                self._vR.put(0)
                if self._pcStart.get():
                    x_act = self._x.get()
                    y_act = self._y.get()
                    if abs(x_act) < 1 and abs(y_act) < 1:
                        self._x_des = self._xDes.get()
                        self._y_des = self._yDes.get()
                        self._psi_des = self._psiDes.get()
                        dx = self._x_des-x_act
                        dy = self._y_des-y_act
                        psi_act = self._psi.get()
                        self._psi_init = np.arctan2(dy,dx)
                        self._state = S1_INIT_POSE

            elif self._state == S1_INIT_POSE:
                psi_act = self._psi.get()
                # print(f"{psi_act},{self._psi_init}")
                err = wrap_angle(self._psi_init - psi_act)
                v_cmd = self._kpsi*err
                if v_cmd>0:
                    v_cmd = min(max(v_cmd,85),250)
                else:
                    v_cmd = max(min(v_cmd,-85),-250)
                if abs(err) > .01:                   
                    self._vL.put(-v_cmd)
                    self._vR.put(v_cmd)
                else:
                    self._vL.put(0)
                    self._vR.put(0)
                    self._state = S2_DRIVE

            elif self._state == S2_DRIVE:
                x_act = self._x.get()
                y_act = self._y.get()
                # print(f"{x_act},{y_act}")
                psi_act = self._psi.get()
                dx = self._x_des - x_act
                dy = self._y_des - y_act
                psi_des = np.arctan2(dy,dx)
                dist = (dx**2 + dy**2)**0.5
                psi_err = wrap_angle(psi_des - psi_act)
                v_forward = min(max(50,dist*self._kdist),250)
                v_steer = psi_err*self._kpsi2
                if dist > 10:
                    self._vL.put(v_forward-v_steer)
                    self._vR.put(v_forward+v_steer)
                else:
                    self._vL.put(0)
                    self._vR.put(0)
                    self._state = S3_FINAL_POSE
            elif self._state == S3_FINAL_POSE:
                psi_act = self._psi.get()
                # print(f"{psi_act},{self._psi_des}")
                err = wrap_angle(self._psi_des - psi_act)
                v_cmd = self._kpsi*err
                if v_cmd>0:
                    v_cmd = min(max(v_cmd,85),250)
                else:
                    v_cmd = max(min(v_cmd,-85),-250)
                if abs(err) > .03:                   
                    self._vL.put(-v_cmd)
                    self._vR.put(v_cmd)
                else:
                    self._vL.put(0)
                    self._vR.put(0)
                    self._pcStart.put(False)
                    self._state = S0_WAIT
            yield self._state

def wrap_angle(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi



                    
                

                

