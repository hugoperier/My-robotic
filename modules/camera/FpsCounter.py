import time
import collections

class FpsCounter:
    def __init__(self, defined_fps):
        self.fps = 0
        self.defined_fps = defined_fps
        self.frametimestamps = collections.deque(maxlen=50)
        self.frametimestamps.append(time.time())

    def wait(self):
        ellasped = (time.time() - self.frametimestamps[-1])
        freq = 1 / self.defined_fps
        if ellasped < freq:
            time.sleep(freq - ellasped)
    
    def reset(self):
        self.fps = 0
        self.frametimestamps = collections.deque(maxlen=50)
        self.frametimestamps.append(time.time())

    def update(self):
        self.frametimestamps.append(time.time())
        if(len(self.frametimestamps) > 1):
            self.fps = int(len(self.frametimestamps)/(self.frametimestamps[-1]-self.frametimestamps[0]))
        else:
            self.fps = 0