import micropython
from task_share import Share
from ulab import numpy as np
from sensor_class import sensor_array
from motor_class import Motor

S0_INIT = micropython.const(1)
S1_SECTION1 = micropython.const(2)
S2_SECTION2 = micropython.const(3)
S3_SECTION3 = micropython.const(4)
S4_SECTION4 = micropython.const(5)
S5_SECTION5 = micropython.const(6)
S6_SECTION6 = micropython.const(7)
S7_SECTION7 = micropython.const(8)

class Final_Path:
    #add to main the class object and add task to task list, make sure to add all shares and queues to it
    def __init__(self,start_run,follow,xPos,yPos,psi,recover_done,xDes,yDes,psiDes,pcStart,leftMotorSetpoint,rightMotorSetpoint,shareEstimator,sens,recover_mot,vL_r,vR_r,motorL,motorR):
        self._recover_mot : Share = recover_mot
        self._state = S0_INIT
        self._start_run : Share = start_run #share to tell overall task to start
        self._follow: Share = follow #share to tell motors to run and use line follower
        self._xPos : Share = xPos
        self._yPos : Share = yPos
        self._psi = psi
        self._recover_done : Share = recover_done #share to inform on bump sensor pressed
        self._xDes : Share = xDes
        self._yDes : Share = yDes
        self._psiDes : Share = psiDes
        self._pcStart : Share = pcStart
        self._leftMotorSetpoint : Share = leftMotorSetpoint
        self._rightMotorSetpoint : Share = rightMotorSetpoint
        self._shareEstimator : Share = shareEstimator
        self._sens : sensor_array = sens
        self._vL_r : Share = vL_r
        self._vR_r : Share = vR_r
        self._motL : Motor = motorL
        self._motR : Motor = motorR
        self._counter = 400

    def run(self):
        while(True):
            if self._state == S0_INIT:
                #Start line follower
                if self._start_run.get():
                    self._motL.enable()
                    self._motR.enable()
                    self._start_run.put(False)
                    self._leftMotorSetpoint.put(300)
                    self._rightMotorSetpoint.put(300)
                    self._state = S1_SECTION1
                    self._follow.put(True) #start line follower here instead of next state

            elif self._state == S1_SECTION1:
                #wait for line folower to lose centroid, then activate pose controller
                if self._sens.centroid() == None:
                    self._shareEstimator.put(True)
                    self._follow.put(False)
                    self._xDes.put(200)
                    self._yDes.put(-100)
                    self._psiDes.put(-(np.pi)/2 - 0.5)
                    self._pcStart.put(True)
                    self._state = S2_SECTION2
                        
            elif self._state == S2_SECTION2:
                if not self._pcStart.get():
                    self._shareEstimator.put(False)
                    self._xDes.put(550)
                    self._yDes.put(0)
                    self._psiDes.put((np.pi/2))
                    self._pcStart.put(True)
                    self._state = S3_SECTION3
                    # go forward until wall is hit, then let task_recover take over
            elif self._state == S3_SECTION3:
                self._shareEstimator.put(True)
                
                if self._recover_done.get():
                    self._shareEstimator.put(False)
                    self._recover_done.put(False)
                    self._follow.put(True)
                    self._leftMotorSetpoint.put(300)
                    self._rightMotorSetpoint.put(300)
                    self._state = S4_SECTION4
                    # when recover task is done, reactivate line follower and set setpoints
                
            elif self._state == S4_SECTION4:
                # When centroid is lost, turn off follow and turn right
                if self._sens.centroid() == None:
                    self._follow.put(False)
                    self._recover_mot.put(True)
                    self._vL_r.put(200)
                    self._vR_r.put(-200)
                    self._state = S5_SECTION5

                
            elif self._state == S5_SECTION5:
            # when centroid is found, re enable line following
                if self._sens.centroid()!=None:
                    self._vL_r.put(0)
                    self._vR_r.put(0)
                    self._recover_mot.put(False)
                    self._follow.put(True)
                    self._leftMotorSetpoint.put(200)
                    self._rightMotorSetpoint.put(200)
                    while self._counter>0:
                        self._counter-=1
                        yield self._state
                    self._counter = 400
                    self._state = S6_SECTION6

            elif self._state == S6_SECTION6:
            # when next centroid is lost again disable line following and set final desired positin and heading
                if self._sens.centroid()==None:
                    self._follow.put(False)
                    self._shareEstimator.put(True)
                    self._xDes.put(-300)
                    self._yDes.put(300)
                    self._psiDes.put(0)
                    self._pcStart.put(True)
                    self._state = S7_SECTION7

            elif self._state == S7_SECTION7:
                if not self._pcStart.get():
                    self._shareEstimator.put(False)
                    self._motL.disable()
                    self._motR.disable()
                    self._state = S0_INIT
            yield self._state

                

