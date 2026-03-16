''' This file demonstrates an example motor task using a custom class with a
    run method implemented as a generator
'''
from motor_class import Motor
from encoder_class import Encoder
from Motor_Control import Motor_Control
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_SET  = micropython.const(2) # State 2 - Set constants if they changed
S3_RUN  = micropython.const(3) # State 3 - run closed loop control


class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
                 mot: Motor, enc: Encoder, control: Motor_Control, 
                 goFlag: Share, dataValues: Queue, timeValues: Queue,
                 KP: Queue, KI: Queue, Setpoint: Queue):
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''

        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: Motor = mot        # A motor object
        
        self._enc: Encoder      = enc        # An encoder object

        self._control: Motor_Control = control
        
        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection
        
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data

        self._KP : Queue = KP

        self._KI : Queue = KI

        self._Setpoint : Queue = Setpoint   #Queue object
        self._CurrentSetpoint: float = 0.0  #Physical # to pass
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data
        
        print("Motor Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                print("Initializing motor task") 
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._goFlag.get():
                    # print("Starting motor loop")
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    self._startTime = ticks_us()
                    self._state = S2_SET  
              
            elif self._state == S2_SET: # Closed-loop control state
                while self._dataValues.any():
                    self._dataValues.get()
                while self._timeValues.any():
                    self._dataValues.get()
                # print("stuck")
                if self._KP.any():
                    # print("changing Kp")
                    self._control.change_gain_KP(self._KP.get())
                if self._KI.any():
                    # print("changing Ki")
                    self._control.change_gain_KI(self._KI.get())
                if self._Setpoint.any():
                    # print("changing setpoint")
                    self._CurrentSetpoint = self._Setpoint.get()
                self._state = S3_RUN
                self._mot.enable() #make motor able to run
        
            elif self._state == S3_RUN:
                # print(f"Running motor loop, cycle {self._dataValues.num_in()}")
                
                self._mot.set_effort(self._control.control(self._CurrentSetpoint))
                
                # Collect a timestamp to use for this sample
                t   = ticks_us()
                

                
                # Store the sampled values in the queues
                self._dataValues.put(self._enc.get_velocity())
                self._timeValues.put(ticks_diff(t, self._startTime))
                
                # When the queues are full, data collection is over
                if self._dataValues.full():
                    # print("Exiting motor loop")
                    self._control.reset() #reset stored I
                    self._state = S1_WAIT
                    self._goFlag.put(False)
                    self._mot.disable() #make motor unable to run
            
            yield self._state