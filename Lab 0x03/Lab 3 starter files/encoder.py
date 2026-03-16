''' This file implements a "dummy" class to use in place of encoder objects
'''

from random import random

class encoder:
    ''' A dummy class that can be instantiated in place of encoder objects
    '''
    
    def __init__(self):
        ''' Initializes an encoder object '''
        # print("Encoder object instantiated")
        self.zero()
        self._position = 0
    
    def update(self, cbSRC = None):
        ''' Update the encoder count. This function is meant to be called
            periodically in a task or using a Timer
        '''
        # print("Encoder updated")
        self._position += int(10*(random()-0.5))
    
    def get_position(self):
        ''' Returns the current position of the encoder
        
        Returns:
            int The current position of the encoder in units of ticks
        '''
        return self._position
    
    def zero(self):
        ''' Zeros the encoder position at the current orientation. Used to
            reestablish a new datum position for the encoder
        '''
        # print("Encoder position zeroed")
        self._position = 0