''' This file demonstrates an example UI task using a custom class with a
    run method implemented as a generator
'''
import pyb
from pyb import USB_VCP, UART
from task_share import BaseShare,Share, Queue
import micropython
from time import sleep

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_CMD  = micropython.const(1) # State 1 - wait for character input
S2_SETK  = micropython.const(2) # State 2 - wait for gain input
S3_SETS  = micropython.const(3) # State 3 - wait for setpoint input
S4_COL  = micropython.const(4) # State 4 - wait for data collection to end
S5_DIS  = micropython.const(5) # State 5 - display the collected data
S6_DISC  = micropython.const(6) # State 6 - display the collected data for centroid
S7_DISP_EST = micropython.const(7) # State 7 - display collected data for state estimator

UI_prompt = ">: "

class task_user:
    '''
    A class that represents a UI task. The task is responsible for reading user
    input over a serial port, parsing the input for single-character commands,
    and then manipulating shared variables to communicate with other tasks based
    on the user commands.
    '''

    def __init__(self, LeftMotorGo, RightMotorGo, dataValuesleft, timeValuesleft, dataValuesright, timeValuesright,
                 Left_Motor_KP, Left_Motor_KI, Left_Motor_Setpoint,
                 Right_Motor_KP, Right_Motor_KI, Right_Motor_Setpoint,
                 setzeros, setnormalize, read, follow, timeValuesSensor, dataValuesCentroid,
                shareEstimator, button, start_run):
        '''
        Initializes a UI task object
        
        Args:
            leftMotorGo (Share):  A share object representing a boolean flag to
                                  start data collection on the left motor
            rightMotorGo (Share): A share object representing a boolean flag to
                                  start data collection on the right motor
            dataValues (Queue):   A queue object used to store collected encoder
                                  position values
            timeValues (Queue):   A queue object used to store the time stamps
                                  associated with the collected encoder data
        '''
        
        self._state: int          = S0_INIT      # The present state
        
        self._leftMotorGo: Share  = LeftMotorGo  # The "go" flag to start data
                                                 # collection from both motor pairs
                                                 # motor and encoder pair

        self._rightMotorGo: Share  = RightMotorGo  # The "go" flag to start data
                                                 # collection from both motor pairs
                                                 # motor and encoder pair
        
        # self._ser = UART(3,115_200) #use this for bluetooth or serial, 3 is btooth and 2 is stlink
        # pyb.repl_uart(self._ser)                     #change repl to go out of btooth
        self._ser: stream         = USB_VCP()    # A serial port object used to
                                                #  read character entry and to
                                                #  print output

        
        self._dataValuesleft: Queue   = dataValuesleft   # A reusable buffer for data
                                                 # collection
        
        self._timeValuesleft: Queue   = timeValuesleft   # A reusable buffer for time
                                                 # stamping collected data

        self._dataValuesright: Queue   = dataValuesright   # A reusable buffer for data
                                                 # collection
        
        self._timeValuesright: Queue   = timeValuesright   # A reusable buffer for time
                                                 # stamping collected data

        self._Left_Motor_KP: Queue   = Left_Motor_KP   # A reusable buffer for KP left
        self._Left_Motor_KI: Queue   = Left_Motor_KI   # A reusable buffer for KI left
        self._Left_Motor_Setpoint: Queue   = Left_Motor_Setpoint   # A reusable buffer for Setpoint left

        self._Right_Motor_KP: Queue   = Right_Motor_KP   # A reusable buffer for KP right
        self._Right_Motor_KI: Queue   = Right_Motor_KI   # A reusable buffer for KI right
        self._Right_Motor_Setpoint: Queue   = Right_Motor_Setpoint   # A reusable buffer for Setpoint right

        self._setzeros: Share = setzeros #Share to set zeros on line sensor
        self._setnormalize: Share = setnormalize #Share to set normal high value in line sensor
        self._read: Share = read #Share to set read flag for line sensor
        self._follow: Share = follow #share to tell motors to run and use line follower
        self._timeValuesSensor: Queue = timeValuesSensor
        self._dataValuesCentroid: Queue = dataValuesCentroid
        self._printcentroidFLG = False

        self._DONE : bool = False # Boolean flag to indicate commands have been receieved
        self._printFLGL : bool = False
        self._printFLGR : bool = False

        self._shareEstimator : Share = shareEstimator
        self._printEstimatorFLG : bool = False

        self.KPbool = False
        self.KIbool = False
        self.Setpointbool = False

        self._CharGen = None
        self._GenFLG = False

        self.button : Share = button
        self._start_run : Share = start_run
    
        self._ser.write("User Task object instantiated\r\n")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                self._ser.write("Initializing user task\r\n")
                self._ser.write("+------------------------------------------------------------------------------+\r\n")
                self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                self._ser.write("| k | Enter new gain values                                                    |\r\n")
                self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                self._ser.write("| l | Run Line Follower                                                        |\r\n")
                self._ser.write("| e | Run State Estimator                                                      |\r\n")
                self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                self._ser.write(UI_prompt)
                self._state = S1_CMD
                
                
            elif self._state == S1_CMD: # Wait for UI commands               
                # Wait for at least one character in serial buffer
                if self._ser.any():
                    # Read the character and decode it into a string
                    inChar = self._ser.read(1).decode()
                    # If the character is an upper or lower case "l", start data
                    # collection on the left motor and if it is an "r", start
                    # data collection on the right motor
                    if inChar in {"g", "G"}:
                        print("GOING")
                        self._ser.write(f"{inChar}\r\n")
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(True)
                        self._ser.write("Starting motor loops...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n")
                        self._state = S4_COL
                    elif inChar in {"k", "K"}:
                        self._state = S2_SETK
                    elif inChar in {"s", "S"}:
                        self._state = S3_SETS
                    elif inChar in {"z", "Z"}:
                        print("Z set")
                        self._setzeros.put(True)
                    elif inChar in {"n", "N"}:
                        print("N set")
                        self._setnormalize.put(True)
                    elif inChar in {"r", "R"}:
                        self._read.put(True)
                    elif inChar in {"l", "L"}:
                        if self._follow.get():
                            self._follow.put(False)
                            self._state = S6_DISC
                            print("stopping")
                        else:
                            self._follow.put(True)
                            print("following")
                    elif inChar in {"e", "E"}:
                        if self._shareEstimator.get():
                            self._shareEstimator.put(False)
                            self._state = S7_DISP_EST
                        else:
                            self._shareEstimator.put(True)
                elif self.button.get():

                    self.button.put(False)
                    print("User button pressed")
                    if not self._start_run.get():
                        self._start_run.put(True)
                        
            elif self._state == S2_SETK:
                if not self._Left_Motor_KP.any() and not self._Right_Motor_KP.any():
                    if not self.KPbool:
                        self._ser.write("Set KP Gain")
                        self._ser.write(UI_prompt)
                        self.KPbool = True
                    if self._CharGen is None:
                        self._CharGen = self.getchar(self._Left_Motor_KP,self._Right_Motor_KP)
                        self._GenFLG = True
                    if self._GenFLG:
                        yield from self._CharGen
                elif not self._CharGen == None: 
                    self._CharGen = None

                if not self._Left_Motor_KI.any() and not self._Right_Motor_KI.any():
                    if not self.KIbool:
                        self._ser.write("Set KI Gain")
                        self._ser.write(UI_prompt)
                        self.KIbool = True
                    if self._CharGen is None:
                        self._CharGen = self.getchar(self._Left_Motor_KI,self._Right_Motor_KI)
                        self._GenFLG = True
                    if self._GenFLG:
                        yield from self._CharGen

                if self._Left_Motor_KI.any() and self._Right_Motor_KI.any():
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
                    self._CharGen = None # set it here instead of in the KI code because it was easier
                    self.KPbool = False
                    self.KIbool = False

            elif self._state == S3_SETS:
                if not self._Left_Motor_Setpoint.any() and not self._Right_Motor_Setpoint.any():
                    if not self.Setpointbool:
                        self._ser.write("Set Setpoint in mm/s")
                        self._ser.write(UI_prompt)
                        self.Setpointbool = True
                    if self._CharGen is None:
                        self._CharGen = self.getchar(self._Left_Motor_Setpoint,self._Right_Motor_Setpoint)
                        self._GenFLG = True
                    if self._GenFLG == True:
                        yield from self._CharGen 
                        
                if self._Left_Motor_Setpoint.any() and self._Right_Motor_Setpoint.any():
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
                    self._CharGen = None
                    self.Setpointbool = False

            elif self._state == S4_COL:
                # While the data is collecting (in the motor task) block out the
                # UI and discard any character entry so that commands don't
                # queue up in the serial buffer
                if self._ser.any(): self._ser.read(1)
                
                # When both go flags are clear, the data collection must have
                # ended and it is time to print the collected data.
                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write("Printing data...\r\n")
                    self._state = S5_DIS

            elif self._state == S5_DIS:
                # While data remains in the buffer, print that data in a command
                # separated format. Otherwise, the data collection is finished.
                if self._printFLGL == False:
                    self._ser.write(f"DIS: left={self._dataValuesleft.num_in()} right={self._dataValuesright.num_in()}\r\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Left Time, Left Position\r\n")
                    self._printFLGL = True
                elif self._dataValuesleft.any():
                    self._ser.write(f"{self._timeValuesleft.get()},{self._dataValuesleft.get()}\r\n")
                elif self._printFLGR == False:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Right Time, Right Position\r\n")
                    self._printFLGR = True
                elif self._dataValuesright.any():
                    self._ser.write(f"{self._timeValuesright.get()},{self._dataValuesright.get()}\r\n")
                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("\r\n")
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._printFLGL = False
                    self._printFLGR = False
                    self._state = S1_CMD

            elif self._state == S6_DISC:
                if self._printcentroidFLG == False:
                    # self._ser.write("Time, Centroid\r\n")
                    self._printcentroidFLG = True
                if self._dataValuesCentroid.any():
                    self._ser.write(f"{self._timeValuesSensor.get()},{self._dataValuesCentroid.get()}\r\n")
                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("\r\n")
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
                    self._printcentroidFLG = False
            
            elif self._state == S7_DISP_EST:
                if not self._printEstimatorFLG:
                    # print("Arc Length [mm], Heading Angle [rad]")
                    self._printEstimatorFLG = True
                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("\r\n")
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
                    self._printEstimatorFLG = False

            yield self._state
    #see multichar_input.py for commented version of this code, itll be a more basic version some changes were made to this one
    def getchar(self, outshare, outshare2):
        out_share = outshare
        out_share2 = outshare2
        char_buf: str      = ""
        digits:   set(str) = set(map(str,range(10)))
        term:     set(str) = {"\r", "\n"}
        done = False
        while not done:
            if self._ser.any():
                char_in = self._ser.read(1).decode()
                if char_in in digits:
                    self._ser.write(char_in)
                    char_buf += char_in
                elif char_in == "." and "." not in char_buf:
                    self._ser.write(char_in)
                    char_buf += char_in
                elif char_in == "-" and len(char_buf) == 0:
                    self._ser.write(char_in)
                    char_buf += char_in
                elif char_in == "\x7f" and len(char_buf) > 0:
                    self._ser.write(char_in)
                    char_buf = char_buf[:-1]
                elif char_in in term:
                    self._ser.write("\r\n")
                    if len(char_buf) == 0:
                        self._ser.write("Value not changed\r\n")
                        char_buf = ""
                    elif char_buf not in {"-", "."}:
                        value = float(char_buf)
                        out_share.put(value)
                        out_share2.put(value)
                        self._ser.write(f"Value set to {value}\r\n")
                    done = True
                    self._GenFLG = False
            yield