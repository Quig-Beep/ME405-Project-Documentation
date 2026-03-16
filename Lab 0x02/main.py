from motor_class import Motor
from encoder_class import Encoder
from pyb import Timer, Pin
from time import ticks_ms, ticks_diff, sleep

motorTimer = Timer(3, freq=20000)
TickTimer  = Timer(4, freq=200)

Left_Motor  = Motor(Pin.cpu.A6, Pin.cpu.B9, Pin.cpu.B8, motorTimer, 1)
Right_Motor = Motor(Pin.cpu.A7, Pin.cpu.B5, Pin.cpu.B4, motorTimer, 2)

Left_Encoder  = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)
Right_Encoder = Encoder(2, Pin.cpu.A0, Pin.cpu.A1)

Left_Motor.enable()
Right_Motor.enable()

printFLG = False

def tim_cb(t):
    global printFLG
    Left_Encoder.update()
    Right_Encoder.update()
    printFLG = True

TickTimer.callback(tim_cb)

# Motor test sequence

state = 0
state_start = ticks_ms()

while True:

    if printFLG:
        printFLG = False
        print(f"{Left_Encoder.get_position()} , {Right_Encoder.get_position()}")

    now = ticks_ms()
    dt = ticks_diff(now, state_start)

    if state == 0:
        Left_Motor.enable()
        Right_Motor.enable()
        Left_Motor.set_effort(50)
        Right_Motor.set_effort(50)
        state_start = now
        state = 1

    elif state == 1 and dt >= 1000:
        Left_Motor.set_effort(100)
        state_start = now
        state = 2

    elif state == 2 and dt >= 1000:
        Right_Motor.set_effort(100)
        state_start = now
        state = 3

    elif state == 3 and dt >= 10000:
        Left_Motor.disable()
        Right_Motor.disable()
        Left_Motor.set_effort(50)
        Right_Motor.set_effort(50)
        state_start = now
        state = 4

    elif state == 4 and dt >= 500:
        Left_Motor.enable()
        Right_Motor.enable()
        state_start = now
        state = 5

    elif state == 5 and dt >= 500:
        Left_Encoder.zero()
        Right_Encoder.zero()
        Left_Motor.set_effort(-50)
        Right_Motor.set_effort(-50)
        state_start = now
        state = 6

    elif state == 6 and dt >= 1000:
        Left_Motor.set_effort(-100)
        state_start = now
        state = 7

    elif state == 7 and dt >= 1000:
        Right_Motor.set_effort(-100)
        state_start = now
        state = 8

    elif state == 8 and dt >= 5000:
        Left_Motor.disable()
        Right_Motor.disable()
        state = 8   # done

    sleep(0.01)

