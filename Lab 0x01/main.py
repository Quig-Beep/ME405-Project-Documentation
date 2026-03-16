from pyb import Pin
from pyb import Timer
from time import sleep_ms

encoderLeft = Timer(1, period = 0xFFFF, prescaler = 0)
encoderLeft.channel(1, pin=Pin(Pin.cpu.A8), mode=Timer.ENC_AB)
encoderLeft.channel(2, pin=Pin(Pin.cpu.A9), mode=Timer.ENC_AB)

encoderRight = Timer(2, period = 0xFFFF, prescaler = 0)
encoderRight.channel(1, pin=Pin(Pin.cpu.A0), mode=Timer.ENC_AB)
encoderRight.channel(2, pin=Pin(Pin.cpu.A1), mode=Timer.ENC_AB)

motorLeftEnable = Pin(Pin.cpu.B8, Pin.OUT_PP)
motorRightEnable = Pin(Pin.cpu.B4, Pin.OUT_PP)
motorLeftDir = Pin(Pin.cpu.B9, Pin.OUT_PP)
motorRightDir = Pin(Pin.cpu.B5, Pin.OUT_PP)
motorTimer = Timer(3, freq=20000)

effortLeft = motorTimer.channel(1, Timer.PWM, pin=Pin(Pin.cpu.A6), pulse_width_percent=50)
effortRight = motorTimer.channel(2, Timer.PWM, pin=Pin(Pin.cpu.A7), pulse_width_percent=50)


motorLeftEnable.high()
motorRightEnable.high()
motorLeftDir.low()
motorRightDir.low()

print(f"{encoderLeft.counter()}, {encoderRight.counter()}")

effortLeft.pulse_width_percent(50)
effortRight.pulse_width_percent(50)
sleep_ms(1000)
print(f"{encoderLeft.counter()}, {encoderRight.counter()}")
effortLeft.pulse_width_percent(100)
effortRight.pulse_width_percent(100)
sleep_ms(1000)
print(f"{encoderLeft.counter()}, {encoderRight.counter()}")
effortLeft.pulse_width_percent(0)
effortRight.pulse_width_percent(0)
motorLeftDir.high()
motorRightDir.high()
effortLeft.pulse_width_percent(50)
effortRight.pulse_width_percent(50)
sleep_ms(1000)
print(f"{encoderLeft.counter()}, {encoderRight.counter()}")
effortLeft.pulse_width_percent(100)
effortRight.pulse_width_percent(100)
sleep_ms(1000)
print(f"{encoderLeft.counter()}, {encoderRight.counter()}")
motorLeftEnable.low()
motorRightEnable.low()
