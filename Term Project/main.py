from pyb import I2C
from time import sleep_ms
from IMU_class import IMU, MODE_IMU
from Motor_Control import Motor_Control
from encoder_class import Encoder
from motor_class import Motor
from task_motor   import task_motor
from pyb import Timer,Pin,ExtInt
from time import sleep_ms, ticks_ms, ticks_diff
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
import gc
from gc           import collect
from task_follow_line import task_follow_line
from task_read_sensor import read_sensors
from task_estimate_state import state_est_task
from sensor_class import sensor_array
from task_sensor import task_sensor
from task_crash import task_crash
from task_recover import task_recover
from Final_Path import Final_Path
from task_pose_control import pose_control
#from task_generate_curve import generate_curve
#rom task_follow_path import follow_path_task

# --- I2C Setup (I2C2 = PB10/PB11 on your board) ---
i2c = I2C(2)
i2c.init(I2C.CONTROLLER, baudrate=400000)
imu = IMU(i2c, addr=0x28)
imu.initialize(mode=MODE_IMU, reset=True, units=0x06)

motorTimer = Timer(3, freq=20000)

Left_Motor  = Motor(Pin.cpu.A6, Pin.cpu.B9, Pin.cpu.B8, motorTimer, 1)
Right_Motor = Motor(Pin.cpu.A7, Pin.cpu.B5, Pin.cpu.B4, motorTimer, 2)

Left_Encoder  = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)
Right_Encoder = Encoder(2, Pin.cpu.A0, Pin.cpu.A1)

Left_Motor.enable()
Right_Motor.enable()

Control_Left = Motor_Control(Left_Motor, Left_Encoder, 0.15, 0.001)
Control_Right = Motor_Control(Right_Motor, Right_Encoder, 0.15, 0.001)

sensors = sensor_array(Pin.cpu.B0,Pin.cpu.B1,Pin.cpu.C0,Pin.cpu.A4,Pin.cpu.C1,Pin.cpu.C10)

pinCrash = Pin.cpu.A15

# Build shares and queues
LeftMotorGo   = Share("B",     name="Go Flag Left")
RightMotorGo  = Share("B",     name="Go Flag Right")
Left_Motor_KP = Queue("f", 1, name="Left Motor KP Value")
Left_Motor_KI = Queue("f", 1, name="Left Motor KI Value")
Left_Motor_Setpoint = Queue("f", 1, name="Setpoint for left motor")
Right_Motor_KP = Queue("f", 1, name="Right Motor KP Value")
Right_Motor_KI = Queue("f", 1, name="Right Motor KI Value")
Right_Motor_Setpoint = Queue("f", 1, name="Setpoint for right motor")
dataValuesleft    = Queue("f", 30, name="Data Collection Buffer left")
timeValuesleft    = Queue("L", 30, name="Time Buffer left")
dataValuesright    = Queue("f", 30, name="Data Collection Buffer right")
timeValuesright    = Queue("L", 30, name="Time Buffer right")
setzeros = Share("B" , name = "zero setter for line sensor")
setnormalize = Share("B" , name="normalizer for line sensor")
read = Share("B" , name="read flag for line sensor")
follow = Share("B" , name="run line follower")
dLeft = Queue('f', 1, name="Left Motor setpoint delta")
dRight = Queue('f', 1, name="Right Motor setpoint delta")
timeValuesSensor    = Queue("L", 300, name="Time Buffer left")
dataValuesCentroid    = Queue("f", 300, name="Data Collection Buffer right")
sL = Share("f", name="Left encoder translational distance [mm]")
sR = Share("f", name="Right encoder translational distance [mm]")
voltageL = Share("f", name="Left Motor Command Voltage")
voltageR = Share("f", name="Right Motor Command Voltage")
yaw = Share("f", name="Heading angle [rad]")
yaw_rate = Share("f", name="Yaw Rate [rad/s]")
shareEstimator = Share("B", name="State Estimation flag")
shareEstimator.put(False)
estS = Share("f",name="Estimate Linear Position")
estPsi = Share("f",name="Estimated heading Angle") 
estOmegaL = Share("f", name="Estimate angular velocity of Left Wheel")
estOmegaR = Share("f", name="Estimate angular velocity for Right Wheel")
crash = Share('B',name="Boolean Flag to detect crash")
button = Share('B',name="Bool flag for user button press")
xPos = Share('f', name="Estimated X position")
yPos = Share('f', name="Estimated Y position")
recover_done = Share("B", name="Finished Recovery Operations")
# xdot = Share('f', name="translational x velocity")
# ydot = Share('f', name="Translational y velocity")
# start_path = Share('B', name="booloean flag to start path follower")
# ax = Share('f', name="ax coefficient")
# bx = Share('f', name="bx coefficient")
# cx = Share('f', name="cx coefficient")
# dx = Share('f', name="dx coefficient")
# ay = Share('f', name="ay coefficient")
# by = Share('f', name="by coefficient")
# cy = Share('f', name="cy coefficient")
# dy = Share('f', name="dy coefficient")
#segment_id = Share('B', name="Segment ID")
vL = Share('f', name="Left Motor Velocity")
vR = Share('f', name="Right Motor Velocity")
# follow_path = Share('B',name='Bool flag for enabling path following')
# next_segment = Share('B',name="Bool flag for starting next segment")
start_run =Share('B',name = "Bool flag for starting the pathing sequence")
changex = Queue('f', 1, name="change state estimator x position")
changey = Queue('f', 1, name="change state estimator y position")
xDes = Share('f', name="desired x position")
yDes = Share('f', name="Desired y position")
psiDes = Share('f', name="Desired final heading")
pcStart = Share('B', name="Bool flag to start pose controller")
recover_mot = Share('B', name="Bool Flag for enabling recovery ops in motor task")
recover_mot.put(False)
vL_r = Share('f', name="recovery task left motor velocity")
vR_r = Share('f', name="recovery task right motor velocity")

# Build task class objects
leftMotorTask  = task_motor(Left_Motor,  Left_Encoder, Control_Left,
                            LeftMotorGo, dataValuesleft, timeValuesleft, 
                            Left_Motor_KP, Left_Motor_KI, Left_Motor_Setpoint,
                            follow, dLeft, vL, pcStart,recover_mot,vL_r)
rightMotorTask = task_motor(Right_Motor, Right_Encoder, Control_Right,
                            RightMotorGo, dataValuesright, timeValuesright,
                            Right_Motor_KP, Right_Motor_KI, Right_Motor_Setpoint,
                            follow, dRight, vR, pcStart,recover_mot, vR_r)
userTask = task_user(LeftMotorGo, RightMotorGo, dataValuesleft, timeValuesleft, dataValuesright, timeValuesright,
                     Left_Motor_KP, Left_Motor_KI, Left_Motor_Setpoint,
                     Right_Motor_KP, Right_Motor_KI, Right_Motor_Setpoint,
                     setzeros, setnormalize, read, follow, timeValuesSensor, dataValuesCentroid, shareEstimator, button, start_run)
sensor_task = task_sensor(sensors, setzeros, setnormalize,read)
line_follow_task = task_follow_line(sensors, dLeft, dRight, timeValuesSensor, dataValuesCentroid, follow)
read_sensors_task = read_sensors(imu,Left_Encoder,Right_Encoder,Left_Motor,Right_Motor,sL,sR,yaw,yaw_rate,voltageL,voltageR)
estimate_state_task = state_est_task(sL,sR,voltageL,voltageR,yaw,yaw_rate,shareEstimator,estS,estPsi,estOmegaL,estOmegaR,xPos,yPos,changex,changey,Left_Encoder,Right_Encoder)
recover_obj = task_recover(crash,recover_done,vL_r,vR_r,shareEstimator,pcStart,sL,sR,yaw,sensors,recover_mot,follow)
recover_task = Task(recover_obj.run, name="Crash recovery manuevers task", priority=5, period=20, profile=True)
crash_task = task_crash(pinCrash,crash,recover_task)
final_path_task = Final_Path(start_run,follow,xPos,yPos,estPsi,recover_done,xDes,yDes,psiDes,pcStart,Left_Motor_Setpoint,Right_Motor_Setpoint,shareEstimator,sensors,recover_mot,vL_r,vR_r,Left_Motor,Right_Motor)
pose_control_task = pose_control(xPos, yPos, estPsi, vL, vR, xDes, yDes, psiDes, pcStart)
#generate_curve_task = generate_curve(estS,estPsi,estOmegaL,estOmegaR,start_path,xPos,yPos,xdot, ydot,ax,bx,cx,dx,ay,by,cy,dy,segment_id,follow_path,next_segment,changex,changey)
#task_follow_path = follow_path_task(vL,vR,ax,bx,cx,dx,ay,by,cy,dy,follow_path,segment_id,next_segment,xPos,yPos,estPsi)

# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 3, period = 35, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 3, period = 35, profile=True))
task_list.append(Task(sensor_task.run, name="Sensor Task",
                      priority = 2, period = 10, profile = True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 1, profile= True))
task_list.append(Task(line_follow_task.run, name="Line Following Task",
                        priority = 2, period = 10, profile=True))
task_list.append(Task(read_sensors_task.run, name="Read Sensors Task", priority=0, period=3,profile=True))
task_list.append(Task(estimate_state_task.run, name="State Estimation Task", priority=1, period=10,profile=True))
task_list.append(recover_task)
task_list.append(Task(crash_task.run, name="Crash Detection Task", priority=0, period=5, profile=True))
task_list.append(Task(final_path_task.run, name="final movements",priority=2,period=20,profile=True))
task_list.append(Task(pose_control_task.run, name="pose controller",priority=2, period = 10, profile = True))
#task_list.append(Task(generate_curve_task.run, name="Spline Curve Generator",priority=4,period=50,profile=True))
#task_list.append(Task(task_follow_path.run, name="Follow predetermined path task",priority=2,period=10,profile=True))

# Run the garbage collector preemptively
collect()

# extint callback function for user button press
def callback(_):
    button.put(True)

pinButton = ExtInt(Pin.cpu.C13,ExtInt.IRQ_FALLING,Pin.PULL_UP,callback)
print(gc.mem_free())
# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()
        
    except KeyboardInterrupt:
        print("Program Terminating")
        print("\n=== TASK PROFILE ===")
        print(task_list)          # <--- prints the profiling table
        Left_Motor.disable()
        Right_Motor.disable()
        break


