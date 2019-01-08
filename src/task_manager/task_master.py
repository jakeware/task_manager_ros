# Copyright 2018 Massachusetts Institute of Technology

import time
import Queue
import psutil
import subprocess
import signal
import shlex

from task_manager import task_minion_model

def signal_handler(sig, frame):
        print('[TaskMaster] Caught SIGINT. Exiting...')
        sys.exit(0)

class TaskMaster(object):
    def __init__(self):
        print "TaskMaster::Constructor"
        self.next_task_id = 0
        self.model = task_minion_model.TaskMinionModel()
        self.processes = {}
        self.task_command_queue = Queue.Queue()
        self.task_config_queue = Queue.Queue()

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

    def ProcessExists(self, task_id):
        if task_id in self.processes:
            return True
        print "[TaskMaster::ProcessExists] Missing id:" + str(task_id)
        return False

    def GetProcessById(self, task_id):
        print "TaskMaster::GetProcessById"
        if self.ProcessExists(task_id):
        	return self.processes[task_id]
        return None

    def PushTaskCommand(self, task_command):
        print "TaskMaster::AddTaskCommand"
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
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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

    def Run(self):
        print "TaskMaster::Run"
        signal.signal(signal.SIGINT, signal_handler)

        while True:
            self.ProcessTaskCommandQueue()
            self.ProcessTaskConfigQueue()
            self.publish_task_config_list(self.model.GetTaskConfigList())
            time.sleep(0.1)