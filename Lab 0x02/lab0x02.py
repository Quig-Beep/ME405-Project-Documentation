from pyb import Pin
from pyb import Timer

#

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''
    
    def __init__(self, PWM, DIR, nSLP, timer, cnl):
        '''Initializes a Motor object'''
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.PWM_pin =  timer.channel(cnl, Timer.PWM, pin=Pin(PWM), pulse_width_percent=0)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)
    
    def set_effort(self, effort):
        effort = max(min(effort, 100), -100)
        if effort >= 0:
            self.DIR_pin.low()
            self.PWM_pin.pulse_width_percent(effort)
        else:
            self.DIR_pin.high()
            self.PWM_pin.pulse_width_percent(-effort)
    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.nSLP_pin.high()
        self.PWM_pin.pulse_width_percent(0)
            
    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.nSLP_pin.low()
        self.PWM_pin.pulse_width_percent(0)