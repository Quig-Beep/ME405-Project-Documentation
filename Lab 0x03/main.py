from Motor_Control import Motor_Control
from encoder_class import Encoder
from motor_class import Motor
from task_motor   import task_motor
from pyb import Timer,Pin
from time import sleep_ms, ticks_ms, ticks_diff
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from gc           import collect

motorTimer = Timer(3, freq=20000)

Left_Motor  = Motor(Pin.cpu.A6, Pin.cpu.B9, Pin.cpu.B8, motorTimer, 1)
Right_Motor = Motor(Pin.cpu.A7, Pin.cpu.B5, Pin.cpu.B4, motorTimer, 2)

Left_Encoder  = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)
Right_Encoder = Encoder(2, Pin.cpu.A0, Pin.cpu.A1)

Left_Motor.enable()
Right_Motor.enable()

# Build shares and queues
leftMotorGo   = Share("B",     name="Left Mot. Go Flag")
rightMotorGo  = Share("B",     name="Right Mot. Go Flag")
dataValues    = Queue("f", 30, name="Data Collection Buffer")
timeValues    = Queue("L", 30, name="Time Buffer")

Control_Left = Motor_Control(Left_Motor, Left_Encoder, 0.3, 0.8)
Control_Right = Motor_Control(Right_Motor, Right_Encoder, 0.3, 0.8)

# Build task class objects
leftMotorTask  = task_motor(Left_Motor,  Left_Encoder, Control_Left,
                            leftMotorGo, dataValues, timeValues)
rightMotorTask = task_motor(Right_Motor, Right_Encoder, Control_Right,
                            rightMotorGo, dataValues, timeValues)
userTask = task_user(leftMotorGo, rightMotorGo, dataValues, timeValues)

# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 1, period = 35, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 1, period = 35, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))

# Run the garbage collector preemptively
collect()

# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()
        
    except KeyboardInterrupt:
        print("Program Terminating")
        Left_Motor.disable()
        Right_Motor.disable()
        break

        