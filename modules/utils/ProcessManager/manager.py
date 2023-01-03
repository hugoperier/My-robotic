import logging
import os
import select
import shlex
import struct
import threading
import time

from .process import Process


def sbool(string):
    return True if string == "True" else False


# TODO: Add documentation
class ProcessManager:
    def __init__(self, log_dir=None, log_frequency=30):
        self.log_dir = log_dir
        self.log_frequency = log_frequency
        self.processes = []
        self._log_cpu = []
        self._log_memory = []
        self._stop = False

    def make_process(self, name, command, dir=".", id=None, initializer=None):
        process = Process(name, command, dir, id, initializer=initializer)
        if process in self.processes:
            raise ValueError("Process already exists")
        process.start(True)
        log = False if self.log_dir == None else True
        self.add_process(process, log, log)
        return process.id
        
    def add_process(self, process, log_cpu=False, log_memory=False):
        """Adds a process to be managed.

        Args:
            process (Process): The process
            log_cpu (bool, optional): True if CPU usage should be tracked. Defaults to False.
            log_memory (bool, optional): True if memory usage should be tracked. Defaults to False.

        Returns:
            bool: True if process wasn't already added
        """
        
        if process in self.processes:
            return False
        self.processes.append(process)
        if log_cpu and process not in self._log_cpu:
            self._log_cpu.append(process)
        if log_memory and process not in self._log_memory:
            self._log_memory.append(process) 
        return True

    def stop_process(self, id, flush = False):
        """Stop a proces"""
        for process in self.processes:
            if process.id == id:
                process.kill()
                if flush:
                    self.rem_process(process)
                return True
        return False
            
    def rem_process(self, process):
        """Removes a process"""
        self.processes.remove(process)
        if process in self._log_cpu:
            self._log_cpu.remove(process)
        if process in self._log_memory:
            self._log_memory.remove(process)
            
    def assert_logdir_exists(self):
        if self.log_dir is None:
            raise ValueError("Log directory wasn't specified")
        if not os.path.isdir(self.log_dir):
            os.mkdir(self.log_dir)
            
    def log_process_cpu(self, process):
        self.assert_logdir_exists()
        log_file = os.path.join(self.log_dir, process.name)
        with open(log_file+"_log_cpu", "ab") as file:
            file.write(struct.pack("d", process.get_cpu_perc()))
            
    def log_process_memory(self, process):
        self.assert_logdir_exists()
        log_file = os.path.join(self.log_dir, process.name)
        with open(log_file+"_log_mem", "ab") as file:
            file.write(struct.pack("d", process.get_mem_usage().bytes))

    def get_process(self, id):
        for process in self.processes:
            if process.id == id:
                return process
        return None
        
    @property
    def has_activeprocesses(self):
        return len(list(filter(lambda p: p.active, self.processes))) >= 1
    
    @property
    def log_period(self):
        return 60 / self.log_frequency

    def start(self):
        self._server_thread = threading.Thread(target=self.main_loop)
        self._server_thread.start()
        self._stop = False

    def stop(self):
        self._stop = True
        self._server_thread.join()
        for process in self.processes:
            process.kill()
        
    def main_loop(self):
        try:
            start = time.time()
            while not self._stop:
                if time.time() - start > self.log_period:
                    
                    start = time.time()
                    for process in self.processes:
                        if process in self._log_memory:
                            self.log_process_memory(process)
                        if process in self._log_cpu:
                            self.log_process_cpu(process)
                        if process.active:
                            process.process_stdout()
                            process.process_stderr()
                time.sleep(0.1)
        except KeyboardInterrupt:    
            pass
        finally:
            self._stop = True
