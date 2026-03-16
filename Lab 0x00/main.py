import pyb
from pyb import Pin
from pyb import ADC
from pyb import Timer
from array import array
from time import sleep
PC0 = Pin(Pin.cpu.C0, Pin.IN)
PC1 = Pin(Pin.cpu.C1, Pin.OUT)

adc = ADC(PC0)

tim = Timer(6, freq = 500)

data = array('H', [0] * 300)
idx = 0
done = False
PC1.low()

def tim_cb(tim):
    global data, idx
    val = adc.read()
    if idx < len(data):
        data[idx] = val
        idx += 1
    else:
        tim.callback(None)
        done = True

# Toggle input and measure step response with ADC
PC1.high()
tim.callback(tim_cb)

while not done:
    pass

for idx, val in enumerate(data):
    print(f"{idx}, {val}")
