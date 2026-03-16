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
        self.Icurrent = 0
        self.Iflg = True
        self.err = 0

    def control(self, Vref):
        # Max velocity is ~ 880 mm/s
        Vref = max(min(Vref, 880), -880)
        self.encoder.update()
        time = ticks_us()
        Vact = self.encoder.get_velocity() # velocity in mm/s
        self.err = Vref-Vact
        # Proportional Effort
        P = self.KP*self.err
        # Integral Effort
        self.dt = ticks_diff(time, self.prev_time)/1000000
        if self.Iflg:
            self.I += self.err*self.dt
            self.Icurrent = self.I*self.KI
        Effort = max(min(P+self.Icurrent, 100), -100)
        if abs(Effort)>=100:
            self.Iflg = False
        else: 
            self.Iflg = True
        self.e_prev = self.err
        return Effort
    
    def reset(self):
        self.I = 0
        self.err = 0
        self.Iflg = True
    
    def change_gain_KP(self, KP_new):
        self.KP = KP_new
    def change_gain_KI(self, KI_new):
        self.KI = KI_new

