import time

class FpsCounter:
    def __init__(self, defined_fps):
        self.fps = 0
        self.lastTime = time.time()
        self.frames = 0
        self.defined_fps = defined_fps

    def wait(self):
        if self.fps > self.defined_fps:
            time.sleep(1 / self.defined_fps - (time.time() - self.lastTime))
    
    def reset(self):
        self.fps = 0
        self.lastTime = time.time()
        self.frames = 0

    def update(self):
        self.frames += 1
        if time.time() - self.lastTime >= 1:
            self.fps = self.frames
            self.frames = 0
            self.lastTime = time.time()
        return self.fps