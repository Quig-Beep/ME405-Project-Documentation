from Motor_Control import Motor_Control
from encoder_class import Encoder
from motor_class import Motor
from pyb import Timer,Pin
from time import sleep_ms, ticks_ms, ticks_diff

motorTimer = Timer(3, freq=20000)
TickTimer  = Timer(4, freq=200)

Left_Motor  = Motor(Pin.cpu.A6, Pin.cpu.B9, Pin.cpu.B8, motorTimer, 1)
Right_Motor = Motor(Pin.cpu.A7, Pin.cpu.B5, Pin.cpu.B4, motorTimer, 2)

Left_Encoder  = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)
Right_Encoder = Encoder(2, Pin.cpu.A0, Pin.cpu.A1)

Left_Motor.enable()
Right_Motor.enable()

Control_Left = Motor_Control(Left_Motor, Left_Encoder, 1, 1)
Control_Right = Motor_Control(Right_Motor, Right_Encoder, 1, 1)

printFLG = False

def tim_cb(t):
    global printFLG
    printFLG = True

TickTimer.callback(tim_cb)

# Motor test sequence

state = 0
state_start = ticks_ms()

while True:

    if printFLG:
        Left_Motor.set_effort(Control_Left.control(Vref_Left))
        Right_Motor.set_effort(Control_Right.control(Vref_Right))
        printFLG = False
        print(f"{Left_Encoder.get_velocity()} , {Right_Encoder.get_velocity()}")

    now = ticks_ms()
    dt = ticks_diff(now, state_start)

    if state == 0:
        Vref_Left = 200
        Vref_Right = 200
        state_start = now
        state = 1
    elif state == 1 and dt >= 1000:
        Vref_Left = 400
        state_start = now
        state = 2
    elif state == 2 and dt >= 1000:
        Vref_Right = 400
        state_start = now
        state = 3
    elif state == 3 and dt >= 1000:
        Vref_Left = 0
        Vref_Right = 0
        state_start = now
        state = 4
    elif state == 4 and dt >= 1000:
        Vref_Left = -200
        Vref_Right = -200
        state_start = now
        state = 5
    elif state == 5 and dt >= 1000:
        Vref_Left = -400
        Vref_Right = -400
        state_start = now
        state = 6
    elif state == 6 and dt >= 1000:
        Vref_Left = 0
        Vref_Right = 0
        state_start = now
        