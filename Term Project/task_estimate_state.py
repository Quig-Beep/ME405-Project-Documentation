from IMU_class import IMU
from encoder_class import Encoder
from ulab import numpy as np
from micropython import const
from task_share import Share, Queue

S0_WAIT = const(0)
S1_RUN = const(1)
S2_SKIP = const(2)

#shareEstimator is a share that sets the state of this task accordingly

class state_est_task:
    def __init__(self, sL, sR, voltageL, voltageR, yaw, yaw_rate, shareEstimator, estS, estPsi, estOmegaL, estOmegaR, xPos, yPos, changex,changey,leftEncoder,rightEncoder):
        self.sL : Share = sL
        self.sR : Share = sR
        self.voltageL : Share = voltageL
        self.voltageR : Share = voltageR
        self.yaw : Share = yaw
        self.yaw_rate :Share = yaw_rate
        self._state = S0_WAIT
        self._shareEst : Share = shareEstimator
        self._estS : Share = estS
        self._estPsi : Share = estPsi
        self._estOmegaL : Share = estOmegaL
        self._estOmegaR : Share = estOmegaR
        self.xPos : Share = xPos
        self.yPos : Share = yPos
        self._leftEncoder : Encoder = leftEncoder
        self._rightEncoder : Encoder = rightEncoder
        self._encLInitial = 0
        self._encRInitial = 0
        self._headingInitial = 0
        self._xhat = np.zeros((4,1), dtype = np.float) # initialize state vector
        self._AD = np.array([[0.7427, 0, 0.2494, 0.2494],[0, 0.0061, 0, 0],[-0.1212, 0, 0.3192, 0.3095],[-0.1212, 0, 0.3095, 0.3192]],dtype = np.float)
        self._BD = np.array([[0.1962, 0.1962, 0.1287, 0.1287, 0, 0],[0, 0, -0.0071, 0.0071, 0.0001, 0.0039],[0.7137, 0.4156, 0.0606, 0.0606, 0, -1.8098],[0.4156, 0.7137, 0.0606, 0.0606, 0, 1.8098]],dtype = np.float)
        self._CD = np.array([[1, -70, 0, 0],[1, 70, 0, 0],[0, 1, 0, 0],[0, 0, -0.25, 0.25]], dtype = np.float)
        self._DD = np.array([[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]], dtype = np.float)
        self._xpos = 0
        self._ypos = 0
        self._changex : Queue = changex
        self._changey : Queue = changey
        self._initialHeading = 0
    def run(self):
        while(True):
            if self._state == S0_WAIT:
                if self._shareEst.get() == True:
                    # reset pose
                    self._xpos = 0
                    self._ypos = 0
                    self._xhat = np.zeros((4,1), dtype = np.float)
                    self._leftEncoder.zero()
                    self._rightEncoder.zero()
                    self.sL.put(0)
                    self.sR.put(0)
                    self.yaw_rate.put(0)
                    self._initialHeading = self.yaw.get()
                    self._estPsi.put(0)
                    self._estS.put(0)
                    self.xPos.put(0)
                    self.yPos.put(0)
                    self._state = S2_SKIP
            elif self._state == S2_SKIP:
                self._state = S1_RUN
            elif self._state == S1_RUN:
                if self._changex.any():
                    self._xpos = self._changex.get()
                if self._changey.any():
                    self._ypos = self._changey.get()
                u_star = np.array([[self.voltageL.get()],[self.voltageR.get()],[self.sL.get()],[self.sR.get()],[self.yaw.get()-self._initialHeading],[self.yaw_rate.get()]], dtype = np.float)
                xhat_new = np.dot(self._AD, self._xhat)+np.dot(self._BD, u_star)
                ds = xhat_new[0,0]-self._xhat[0,0]
                self._xpos += ds*np.cos(xhat_new[1,0])
                self._ypos += ds*np.sin(xhat_new[1,0])
                self.xPos.put(self._xpos)
                self.yPos.put(self._ypos)
                self._xhat = xhat_new
                self._estS.put(float(xhat_new[0,0]))
                self._estPsi.put(float(xhat_new[1,0]))
                self._estOmegaL.put(float(xhat_new[2,0]))
                self._estOmegaR.put(float(xhat_new[3,0]))
                if self._shareEst.get() == False:
                    self._state = S0_WAIT
                # if not self.dataValsArc.full():
                #     self.dataValsArc.put(float(xhat_new[0,0]))
                # if not self.dataValsPsi.full():
                #     self.dataValsPsi.put(float(xhat_new[1,0]))
            yield self._state