from sensor_class import sensor_array
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
from pyb import Pin
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_READ = micropython.const(1) # State 1 - read values

class task_sensor:
    def __init__(self,sensors,setzeros,setnormalize,read):
        self._state: int = S0_INIT
        self._sensor_array = sensors
        self._setzeros: Share = setzeros #Share to set zeros on line sensor
        self._setnormalize: Share = setnormalize #Share to set normal high value in line sensor
        self._read: Share = read
        self.centroid = 0

        print("Sensor object instantiated")
    
    def run(self):
        while True:
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                self._state = S1_READ

            if self._state == S1_READ: # Init state (can be removed if unneeded)
                if self._setzeros.get():
                    self._sensor_array.zero()
                    self._setzeros.put(False)
                elif self._setnormalize.get():
                    self._sensor_array.normalize()
                    self._setnormalize.put(False)
                elif self._read.get():
                    vals = self._sensor_array.readlevel()
                    c = self._sensor_array.centroid()
                    print(f"{vals[0]},{vals[1]},{vals[2]},{vals[3]},{vals[4]},{c}")
                    self._read.put(False)
            yield self._state
            