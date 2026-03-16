from pyb import I2C
from time import sleep_ms
from IMU_class import IMU, MODE_IMU
from Motor_Control import Motor_Control
from encoder_class import Encoder
from motor_class import Motor
from task_motor   import task_motor
from pyb import Timer,Pin,ADC
from time import sleep_ms, ticks_ms, ticks_diff
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from gc           import collect
from task_follow_line import task_follow_line
from task_read_sensor import read_sensors
from task_estimate_state import state_est_task
from sensor_class import sensor_array
from task_sensor import task_sensor

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

Control_Left = Motor_Control(Left_Motor, Left_Encoder, 0.067, 0.0005)
Control_Right = Motor_Control(Right_Motor, Right_Encoder, 0.067, 0.0005)

sensors = sensor_array(Pin.cpu.B0,Pin.cpu.B1,Pin.cpu.C0,Pin.cpu.A4,Pin.cpu.C1,Pin.cpu.C10)

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
estS = Share("f",name="Estimate Linear Position")
estPsi = Share("f",name="Estimated heading Angle") 
estOmegaL = Share("f", name="Estimate angular velocity of Left Wheel")
estOmegaR = Share("f", name="Estimate angular velocity for Right Wheel")
dataValsArc = Queue("f", 500, name="Queue for Estimated Arc length [mm]")
dataValsPsi = Queue("f", 500, name="Queue for Estimated heading angle [rad]")

# Build task class objects
leftMotorTask  = task_motor(Left_Motor,  Left_Encoder, Control_Left,
                            LeftMotorGo, dataValuesleft, timeValuesleft, 
                            Left_Motor_KP, Left_Motor_KI, Left_Motor_Setpoint,
                            follow, dLeft)
rightMotorTask = task_motor(Right_Motor, Right_Encoder, Control_Right,
                            RightMotorGo, dataValuesright, timeValuesright,
                            Right_Motor_KP, Right_Motor_KI, Right_Motor_Setpoint,
                            follow, dRight)
userTask = task_user(LeftMotorGo, RightMotorGo, dataValuesleft, timeValuesleft, dataValuesright, timeValuesright,
                     Left_Motor_KP, Left_Motor_KI, Left_Motor_Setpoint,
                     Right_Motor_KP, Right_Motor_KI, Right_Motor_Setpoint,
                     setzeros, setnormalize, read, follow, timeValuesSensor, dataValuesCentroid, shareEstimator, dataValsArc, dataValsPsi)
sensor_task = task_sensor(sensors, setzeros, setnormalize,read)
line_follow_task = task_follow_line(sensors, dLeft, dRight, timeValuesSensor, dataValuesCentroid, follow)
read_sensors_task = read_sensors(imu,Left_Encoder,Right_Encoder,Left_Motor,Right_Motor,sL,sR,yaw,yaw_rate,voltageL,voltageR)
estimate_state_task = state_est_task(sL,sR,voltageL,voltageR,yaw,yaw_rate,shareEstimator,estS,estPsi,estOmegaL,estOmegaR,dataValsArc,dataValsPsi)


# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 3, period = 35, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 3, period = 35, profile=True))
task_list.append(Task(sensor_task.run, name="Sensor Task",
                      priority = 2, period = 10, profile = True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 3, profile= True))
task_list.append(Task(line_follow_task.run, name="Line Following Task",
                        priority = 2, period = 10, profile=True))
task_list.append(Task(read_sensors_task.run, name="Read Sensors Task", priority=0, period=3,profile=True))
task_list.append(Task(estimate_state_task.run, name="State Estimation Task", priority=1, period=20,profile=True))
# Run the garbage collector preemptively
collect()

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


