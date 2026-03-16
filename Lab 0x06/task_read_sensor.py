from IMU_class import IMU
from encoder_class import Encoder
from task_share import Share
from micropython import const
from motor_class import Motor

S1_RUN = const(1)

class read_sensors:
    def __init__(self, IMU, EncoderL, EncoderR, MotorL, MotorR, sL, sR, yaw, yaw_rate, voltageL, voltageR):
        self.IMU : IMU = IMU
        self.EncoderL : Encoder = EncoderL
        self.EncoderR : Encoder = EncoderR
        self.MotorL : Motor = MotorL
        self.MotorR: Motor = MotorR
        self.sL : Share = sL
        self.sR : Share = sR
        self.yaw : Share = yaw
        self.yaw_rate : Share = yaw_rate
        self.voltageL : Share = voltageL
        self.voltageR : Share = voltageR
        self._state = S1_RUN
    def run(self):
        while(True):
            if self._state == S1_RUN:
                self.sL.put(self.EncoderL.get_position())
                self.sR.put(self.EncoderR.get_position())
                self.yaw.put(self.IMU.euler()[0])
                self.yaw_rate.put(self.IMU.ang_v()[2])
                self.voltageL.put(self.MotorL.get_voltage())
                self.voltageR.put(self.MotorR.get_voltage()) 
            yield self._state
