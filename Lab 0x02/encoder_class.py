from pyb import Pin
from pyb import Timer
from time import ticks_us, ticks_diff   # Use to get dt value in update()

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, timer, chA_pin, chB_pin):
        '''Initializes an Encoder object'''
        self.encoderTimer = Timer(timer, period = 0xFFFF, prescaler = 0)
        self.encoderTimer.channel(1, pin=Pin(chA_pin), mode = Timer.ENC_AB)
        self.encoderTimer.channel(2, pin=Pin(chB_pin), mode = Timer.ENC_AB)
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = self.encoderTimer.counter()     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        self.prev_time  = ticks_us()

        # track overflow errors
        self.AR = 65535
        self.overflow = -(self.AR+1)/2
        self.underflow = (self.AR+1)/2
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        count = self.encoderTimer.counter()
        time = ticks_us()
        delta = count-self.prev_count
        if delta < self.overflow:
            delta += (self.AR+1)
        elif delta > self.underflow:
            delta -= (self.AR+1)

        self.position += delta
        self.delta = delta
        self.dt = ticks_diff(time, self.prev_time)
        
        self.prev_time = time
        self.prev_count = count

    def get_position(self):
        return self.position
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        if self.dt == 0:
            return 0
        return self.delta/self.dt
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0
        self.prev_count = self.encoderTimer.counter()
        self.delta = 0
        self.dt = 0
        self.prev_time = ticks_us()