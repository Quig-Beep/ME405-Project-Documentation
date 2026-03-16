from pyb import ExtInt, Pin
from task_share import Share
class task_crash():
    def __init__(self, pin, crash, recover_task):
        # pin used is PA15
        # Initialize interrupt pin, crash share
        self._switch = ExtInt(pin,ExtInt.IRQ_FALLING,Pin.PULL_UP,self.callback)
        self.crash : Share = crash
        self._recover = recover_task
        self._countdown = 0
    def callback(self,_):
        self._switch.disable()
        self.crash.put(True, in_ISR=True)
        self._countdown = 200
    def run(self):
        while True:
            if self._countdown>0:
                self._countdown -= 1
                if self._countdown == 0:
                    self._switch.enable()
            yield