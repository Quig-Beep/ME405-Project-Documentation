''' This file implements a "dummy" class to use in place of motor driver objects
'''

class motor_driver:
    ''' A dummy class that can be instantiated inplace of motor driver objects
    '''
    
    def __init__(self):
        ''' Initializes a motor driver object'''
        # print("Motor object instantiated")
        pass
    
    def enable(self):
        ''' Enables/wakes up a motor driver'''
        # print("Enabling motor")
        pass
    
    def disable(self):
        ''' Disables/puts to sleep a motor driver'''
        # print("Disabling motor")
        pass
    
    def set_effort(self, effort):
        ''' Sets the effort of a motor driver
        
        Args:
            effort (float): The desired motor effort as a signed percentage
                            (+/- 100%)
        '''
        # print(f"Setting motor effort to {effort}%")
        pass