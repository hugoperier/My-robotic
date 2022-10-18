import datetime
import os
import subprocess
import tempfile
import threading
from signal import SIGINT
import psutil

from .units import Size, Time


class Process:
    def __init__(self, name, command, dir=".", id=None, initializer=None):
        if (id == None):
            self.id = id(self)
        else:
            self.id = id
        self.max_buff_size = 10000
        self.name = name
        self._command = command
        self._process = None
        self._start = Time(0)
        self._cpu_usage = 0
        self._thread = None
        self._outstream = None
        self._errstream = None
        self.initialized = False
        self._outbuff = b""
        self._errbuff = b""
        self.line = ""
        self.initializer = initializer
        self._dir = dir
        print(self._dir)
        
    def __eq__(self, other):
        return isinstance(other, Process) and other.name == self.name
        
    def start(self, pipe=False):
        previous = os.path.abspath(os.curdir)
        os.chdir(self._dir)
        if self.active:
            raise OSError("Process is already running")
        self._start = datetime.datetime.now()
        if pipe:
            self._outstream = tempfile.TemporaryFile()
            self._errstream = tempfile.TemporaryFile()
            print("starting popen with")
            print(self._command.split())
            self._process = subprocess.Popen(self._command.split(),
                                             stdout=subprocess.PIPE,
                                             stderr=subprocess.PIPE)
        else:
            print("starting popen with")
            print(self._command.split())
            self._process = subprocess.Popen(self._command.split())
        os.chdir(previous)
            
    @property
    def stdout(self):
        return self._outbuff
    
    @property
    def stderr(self):
        return self._errbuff

    def waitForReady(self):
        if (self.initializer is None):
            print("no initialiser")
            return True
        from time import sleep
        attempts = 0
        maxAttempts = 70
        while not self.initialized:
            print("not ready")
            print(self.line)
            if self.initializer in self.line:
                self.initialized = True
                return True
            if attempts > maxAttempts:
                self.initialized = False
                return False
            attempts += 1
            sleep(0.25)
        if not self.initialized:
            print("Initialisation failed...")
            
    def process_stdout(self):
        line = self._process.stdout.readline()
        print(line)
        if line:
            self.line = line.decode("utf-8")
            print(self.line)
            
    def process_stderr(self):
        line = self._process.stdout.readline()
        print(line)
        if line:
            self.line = line.decode("utf-8")
            print(self.line)
        
    def kill(self):
        self._start = Time(0)
        if (self._process is not None):
            self._process.send_signal(SIGINT)
            self._process.wait(timeout=10)
            self._process.kill()
            self._process.terminate()
            self._process.communicate()
            del self._process 
            self._process = None
        if (self._outstream is not None):
            self._outstream.close()
        if (self._errstream is not None):
            self._errstream.close()
        
        self.initialized = False
        print("Killed process " + self.id)

    def get_info(self):
        return {
            "cpu": self.get_cpu_perc(),
            "mem": self.get_mem_perc(),
            "mem_usage": self.get_mem_usage().kbytes,
            "active": self.active,
            "pid": self.pid,
            "name": self.name,
            "id": self.id
        }
        
    def update_cpu(self):
        try:
            self._cpu_usage = psutil.Process(self.pid).cpu_percent(0.5) / psutil.cpu_count()
        except psutil.NoSuchProcess:
            pass
        
    @property
    def command(self):
        return self._command
        
    @property
    def active(self):
        if self._process is None:
            return False
        return self._process.poll() is None
    
    @property 
    def pid(self):
        if self.active:
            return self._process.pid
        return -1
    
    @property
    def uptime(self):
        if self.active:
            return Time(datetime.datetime.now()-self._start)
        else:
            return Time(0)
    
    def get_mem_usage(self):
        if self.active:
            return Size(psutil.Process(self.pid).memory_info().vms)
        else:
            return Size(0)
    
    def get_mem_perc(self):
        if self.active:
            return psutil.Process(self.pid).memory_percent("vms")
        else:
            return 0
    
    def get_cpu_perc(self):
        if self.active:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self.update_cpu)
                self._thread.setDaemon(True)
                self._thread.start()
            return self._cpu_usage
        else:
            return 0
