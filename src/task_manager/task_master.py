# Copyright 2018 Massachusetts Institute of Technology

import time
import Queue
import signal
import shlex
import sys
import subprocess

from task_manager import task_minion_model
from task_manager import task_info_manager

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
        self.task_command_queue = Queue.Queue()
        self.task_config_queue = Queue.Queue()
        self.task_info_manager = task_info_manager.TaskInfoManager()

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
        self.task_info_manager.AddProcess(task_config.id, process)
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

    def UpdateTaskInfo(self):
        # print "TaskMaster::UpdateTaskInfo"
        for task_id, proc in self.processes.iteritems():
            print "Getting info for task id:" + str(task_id)
            task_info = self.task_info_manager.GetTaskInfoById(task_id)
            if not task_info:
                # print "Got empty task_info"
                continue

            print "Calling SetTaskInfo"
            print "cpu_percent: " + str(task_info.load)
            print "memory_percent: " + str(task_info.memory)
            print "status: " + task_info.status

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