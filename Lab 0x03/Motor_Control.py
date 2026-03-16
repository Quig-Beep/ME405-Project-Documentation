# Motor Control Class P+I
from time import ticks_us, ticks_diff
import encoder_class
import motor_class

class Motor_Control:
    def __init__(self, motor, encoder, KP, KI):
        self.state = 0
        self.KP = KP
        self.KI = KI
        self.motor = motor
        self.encoder = encoder
        self.e_prev = 0
        self.prev_time = ticks_us()
        self.dt = 0
        self.I = 0

    def control(self, Vref):
        # Max velocity is ~ 550 mm/s
        Vref = max(min(Vref, 500), -500)
        self.encoder.update()
        time = ticks_us()
        Vact = self.encoder.get_velocity() # velocity in mm/s
        err = Vref-Vact
        # Proportional Effort
        P = self.KP*err
        # Integral Effort
        self.dt = ticks_diff(time, self.prev_time)/1000000
        self.I += self.KI*0.5*(err + self.e_prev)*self.dt
        self.I = max(min(self.I, 100), -100)
        Effort = max(min(P+self.I, 100), -100)
        self.e_prev = err
        self.prev_time = time
        return Effort
        