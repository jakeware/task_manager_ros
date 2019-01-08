# Copyright 2018 Massachusetts Institute of Technology

import time
import Queue
import psutil
import subprocess
import signal
import shlex
import io
import sys
from threading import Thread

from task_manager import task_minion_model

ON_POSIX = 'posix' in sys.builtin_module_names

def signal_handler(sig, frame):
        print('[TaskMaster] Caught SIGINT. Exiting...')
        sys.exit(0)

class TaskMaster(object):
    def __init__(self):
        print "TaskMaster::Constructor"
        self.next_task_id = 0
        self.model = task_minion_model.TaskMinionModel()
        self.model.SetTaskInfoChangedCallback(self.TaskInfoChanged)
        self.processes = {}
        self.process_outputs = {}
        self.task_command_queue = Queue.Queue()
        self.task_config_queue = Queue.Queue()

    def SetPublishTaskInfoCallback(self, callback):
        self.publish_task_info = callback

    def TaskInfoChanged(self, task_info):
        print "TaskMaster::TaskInfoChanged"
        self.publish_task_info(task_info)

    def SetPublishTaskConfigListCallback(self, callback):
    	self.publish_task_config_list = callback

    def GetNewTaskId(self):
    	task_id = self.next_task_id
    	self.next_task_id = self.next_task_id + 1
    	return task_id

    def PushTaskConfig(self, task_config):
        print "TaskMaster::AddTaskConfig"
        self.task_config_queue.put(task_config)

    def PopTaskConfig(self):
        return self.task_config_queue.get(0)

    def AddProcess(self, task_id, process):
        print "TaskMaster::AddProcess"
        self.processes[task_id] = process

    def AddProcessOutput(self, task_id, process_output):
        print "TaskMaster::AddProcessOutput"
        self.process_outputs[task_id] = process_output

    def ProcessExists(self, task_id):
        if task_id in self.processes:
            return True
        print "[TaskMaster::ProcessExists] Missing id:" + str(task_id)
        return False

    def ProcessOutputExists(self, task_id):
        if task_id in self.process_outputs:
            return True
        print "[TaskMaster::ProcessOutputExists] Missing id:" + str(task_id)
        return False

    def GetProcessById(self, task_id):
        print "TaskMaster::GetProcessById"
        if self.ProcessExists(task_id):
        	return self.processes[task_id]
        return None

    def GetProcessOutputById(self, task_id):
        print "TaskMaster::GetProcessOutputById"
        if self.ProcessOutputExists(task_id):
            return self.process_outputs[task_id]
        return None

    def PushTaskCommand(self, task_command):
        print "TaskMaster::PushTaskCommand"
        self.task_command_queue.put(task_command)

    def PopTaskCommand(self):
        return self.task_command_queue.get(0)

    def ExecuteTaskCommand(self, task_command):
        print "TaskMaster::ExecuteTaskCommand"
        if task_command.command == 'start':
            task_config = self.model.GetTaskConfigById(task_command.id)
            self.StartProcess(task_config)
        elif task_command.command == 'stop':
    		self.StopProcess(task_command.id)

    def StartProcess(self, task_config):
        print "TaskMaster::StartTask"
        cmd = shlex.split(task_config.command)
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=1, close_fds=ON_POSIX)
        output_queue = Queue.Queue()
        output_thread = Thread(target=self.EnqueueProcessOutput, args=(process.stdout, output_queue))
        output_thread.daemon = True  # thread dies with the program
        output_thread.start()
        self.AddProcessOutput(task_config.id, output_queue)
        self.AddProcess(task_config.id, process)

    def StopProcess(self, task_id):
        print "TaskMaster::StopTask"
        process = self.GetProcessById(task_id)
        if process:
        	process.kill()
        	return True
        print "[TaskMaster::StopTask] Failed to stop process with task_id:" + str(task_id) + " and pid:" + str(process.pid) 
        return False

    def ProcessTaskCommandQueue(self):
        while self.task_command_queue.qsize():
            try:
                task_command = self.PopTaskCommand()
                self.ExecuteTaskCommand(task_command)
            except Queue.Empty:
                pass

    def ProcessTaskConfigQueue(self):
        while self.task_config_queue.qsize():
            try:
                task_config = self.PopTaskConfig()
                self.model.AddTaskFromConfig(task_config)
            except Queue.Empty:
                pass

    def EnqueueProcessOutput(self, output, queue):
        print "EnqueueProcessOutput"
        for line in iter(output.readline, b''):
            queue.put(line)
        output.close()

    def UpdateTaskInfo(self):
        print "TaskMaster::UpdateTaskInfo"
        for task_id, proc in self.processes.iteritems():
            print "Getting info for task id:" + str(task_id)

            process_output = self.GetProcessOutputById(task_id)
            if not process_output:
                pass

            task_info = task_minion_model.TaskInfo(task_id)
            try:
                task_info.stdout_delta = process_output.get_nowait() # or q.get(timeout=.1)
                print "stdout_delta: " + task_info.stdout_delta
            except Queue.Empty:
                print "No output for process with task id:" + str(task_id)

            print "Calling SetTaskInfo"
            self.model.SetTaskInfo(task_info)

    def Run(self):
        print "TaskMaster::Run"
        signal.signal(signal.SIGINT, signal_handler)

        while True:
            self.ProcessTaskCommandQueue()
            self.ProcessTaskConfigQueue()
            self.UpdateTaskInfo()
            self.publish_task_config_list(self.model.GetTaskConfigList())
            time.sleep(0.1)