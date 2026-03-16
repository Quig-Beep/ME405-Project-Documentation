# Line following Task
from sensor_class import sensor_array
from encoder_class import Encoder
from task_share import Queue
import micropython
from utime        import ticks_us, ticks_diff


S1_WAIT = micropython.const(1)
S2_RUN = micropython.const(2)

class task_follow_line:
    def __init__(self, sens: sensor_array, dLeft: Queue, dRight: Queue, timeValuesSensor: Queue, dataValuesCentroid: Queue, follow: Queue):
        self._sensor_array = sens
        self._KP = 100
        self._dLeft = dLeft
        self._dRight = dRight

        self._startTime = 0
        self._timeValuesSensor = timeValuesSensor
        self._dataValuesCentroid = dataValuesCentroid
        self._follow = follow

        self._state = S1_WAIT

    def run(self):
        while(True):
            if self._state == S1_WAIT:
                if self._follow.get():
                    self._state = S2_RUN
                    self._startTime = ticks_us()

            elif self._state == S2_RUN:
                if not self._follow.get():
                    self._state = S1_WAIT
                else:
                    centroid = self._sensor_array.centroid()
                    if centroid is not None:
                        if abs(centroid)>.04:
                            u = centroid*self._KP
                            if self._dLeft.any():
                                    self._dLeft.get()
                            if self._dRight.any():
                                    self._dRight.get()
                            if centroid>0:
                                self._dLeft.put(u)
                                self._dRight.put(-u)
                            else:
                                self._dLeft.put(u)
                                self._dRight.put(-u)
                    # t   = ticks_us()
                    # # Store the sampled values in the queues
                    # if not self._dataValuesCentroid.full():
                    #     if centroid != None:
                    #         self._dataValuesCentroid.put(centroid)
                    #     else:
                    #         self._dataValuesCentroid.put(0.0)
                    # if not self._timeValuesSensor.full():
                    #     self._timeValuesSensor.put(ticks_diff(t, self._startTime))
            yield self._state
                


                    
