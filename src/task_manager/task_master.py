# Copyright 2018 Massachusetts Institute of Technology

import time
import Queue
import signal
import shlex
import sys
import subprocess
import signal
import os

from task_manager import task_info_manager
from task_manager import task_manager_core

def signal_handler(sig, frame):
        print('[TaskMaster] Caught SIGINT. Exiting...')
        sys.exit(0)

class TaskMaster(object):
    def __init__(self):
        print "TaskMaster::Constructor"
        self.next_task_id = 0
        self.task_configs = {}
        self.processes = {}
        self.task_command_queue = Queue.Queue()
        self.task_config_queue = Queue.Queue()
        self.task_info_manager = task_info_manager.TaskInfoManager()

    def SetPublishTaskInfoCallback(self, callback):
        self.publish_task_info = callback

    def SetPublishTaskConfigListCallback(self, callback):
    	self.publish_task_config_list = callback

    def GetNewTaskId(self):
    	task_id = self.next_task_id
    	self.next_task_id = self.next_task_id + 1
    	return task_id

    def PushTaskConfig(self, task_config):
        # print "TaskMaster::PushTaskConfig"
        self.task_config_queue.put(task_config)

    def PopTaskConfig(self):
        return self.task_config_queue.get(0)

    def SetProcess(self, task_id, process):
        # print "TaskMaster::AddProcess"
        self.processes[task_id] = process

    def DeleteProcess(self, task_id):
        del self.processes[task_id]

    def ProcessExists(self, task_id):
        if task_id in self.processes:
            return True
        print "[TaskMaster::ProcessExists] Missing id:" + str(task_id)
        return False

    def GetProcessById(self, task_id):
        # print "TaskMaster::GetProcessById"
        if self.ProcessExists(task_id):
        	return self.processes[task_id]
        return None

    def PushTaskCommand(self, task_command):
        # print "TaskMaster::PushTaskCommand"
        self.task_command_queue.put(task_command)

    def PopTaskCommand(self):
        return self.task_command_queue.get(0)

    def TaskConfigExists(self, task_id):
        if task_id in self.task_configs:
            return True
        print "[TaskMaster::TaskConfigExists] Missing id:" + str(task_id)
        return False

    def GetTaskConfigById(self, task_id):
        if self.TaskConfigExists(task_id):
            return self.task_configs[task_id]
        return None

    def SetTaskConfig(self, task_config):
        self.task_configs[task_config.id] = task_config

    def ExecuteTaskCommand(self, task_command):
        # print "TaskMaster::ExecuteTaskCommand"
        if task_command.command == 'start':
            task_config = self.GetTaskConfigById(task_command.id)
            self.StartProcess(task_config)
        elif task_command.command == 'stop':
    		self.StopProcess(task_command.id)

    def StartProcess(self, task_config):
        # print "TaskMaster::StartTask"
        stdbuf_cmd = 'stdbuf -o L'
        cmd = shlex.split(stdbuf_cmd + ' ' + task_config.command)
        print cmd
        if self.ProcessExists(task_config.id):
            print "[TaskMaster::StartProcess] Process with task_id:" + str(task_config.id) + " already exists.  Not starting."
            return
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=1)
        print "[TaskMaster::StartProcess] Started task: " + task_config.name + " with task_id:" + str(task_config.id) + " and pid:" + str(process.pid)
        self.task_info_manager.SetProcess(task_config.id, process)
        self.SetProcess(task_config.id, process)

    def StopProcess(self, task_id):
        print "TaskMaster::StopTask"
        process = self.GetProcessById(task_id)
        if process:
            try:
                process.send_signal(signal.SIGINT)  # seems to work for nodes and launch files!
                # os.system('rosnode kill /test_node1')  # dirty hack, but works
                # process.kill()  # kills ros nodes and launch files too quickly
                # process.terminate()  # kills ros nodes and launch files too quickly
            except:
                print "[TaskMaster::StopProcess] Failed to stop process with pid:" + str(process.pid)
                pass
            return True
        print "[TaskMaster::StopTask] Process not found.  Failed on task_id:" + str(task_id)
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
                self.SetTaskConfig(task_config)
            except Queue.Empty:
                pass

    def UpdateTaskInfo(self):
        # print "TaskMaster::UpdateTaskInfo"
        task_id_list = self.GetTaskConfigKeys()
        for task_id in task_id_list:
            task_info = self.task_info_manager.GetTaskInfoById(task_id)
            if task_info:
                self.publish_task_info(task_info)
                continue

            if not self.task_info_manager.TaskInfoQueueExists(task_id):
                task_info = task_manager_core.TaskInfo(task_id)
                task_info.status = 'stopped'
                self.publish_task_info(task_info)

    def GetProcessesKeys(self):
        return list(self.processes.keys())

    def GetTaskConfigKeys(self):
        return list(self.task_configs.keys())

    def PruneProcesses(self):
        task_id_list = self.GetProcessesKeys()
        for task_id in task_id_list:
            process_pid = self.GetProcessById(task_id).pid
            if not self.task_info_manager.GetProcessIsRunning(process_pid):
                if not self.task_info_manager.TaskQueuesEmpty(task_id):
                    continue
                print "[TaskMaster::PruneProcesses] Deleting process with task_id:" + str(task_id) + " and pid:" + str(process_pid)
                self.DeleteProcess(task_id)
                self.task_info_manager.RemoveTask(task_id)

    def GetTaskConfigList(self):
        task_config_list = []
        for task_config in self.task_configs.itervalues():
            task_config_list.append(task_config)
        return task_config_list

    def Run(self):
        print "TaskMaster::Run"
        signal.signal(signal.SIGINT, signal_handler)

        while True:
            self.ProcessTaskCommandQueue()
            self.ProcessTaskConfigQueue()
            self.UpdateTaskInfo()
            self.PruneProcesses()
            self.publish_task_config_list(self.GetTaskConfigList())
            time.sleep(0.1)