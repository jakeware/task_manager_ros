# Copyright 2018 Massachusetts Institute of Technology

import sys
import os
import yaml
import time
import signal

from Tkinter import *
from ScrolledText import ScrolledText

from task_minion_view.task_minion_view import *
from task_minion_model.task_minion_model import *

def signal_handler(sig, frame):
        print('[TaskMinion] Caught SIGINT. Exiting...')
        sys.exit(0)

class TaskMinionController:
    def __init__(self):
        self.root = Tk()
        self.view = TaskMinionView(self.root)
        self.model = TaskMinionModel()
        self.model.SetTaskStatusCallback(self.TaskStatusChanged)
        self.model.SetProcessConfigCallback(self.TasksChanged)

        self.received_master_process_config = False  # have we received a process config from TaskMaster yet?
        self.active_index = 0
        self.last_active_index = 0

        # bind handlers to input keys
        self.root.bind('<Up>', self.HandleUp)
        self.root.bind('<Down>', self.HandleDown)
        self.root.bind('<Control-s>', self.HandleStart)
        self.root.bind('<Control-k>', self.HandleStop)

    def SetTaskActivity(self):
        if self.last_active_index != self.active_index:
            last_active_task = self.GetTaskByIndex(self.last_active_index)
            self.view.SetTaskInactiveById(last_active_task.id)
            self.SetSubTreeActivity(self.view.SetTaskInactiveById, last_active_task.children)
            active_task = self.GetTaskByIndex(self.active_index)
            self.view.SetTaskAndOutputActiveById(active_task.id)
            self.SetSubTreeActivity(self.view.SetTaskActiveById, active_task.children) 

    def HandleUp(self, event):
        print "HandleUp"
        self.last_active_index = self.active_index
        self.active_index = max(self.active_index - 1, 0)
        print "active_index: " + str(self.active_index)
        self.SetTaskActivity()

    def HandleDown(self, event):
        print "HandleDown"
        self.last_active_index = self.active_index
        self.active_index = min(self.active_index + 1, self.view.GetTaskEntryCount() - 1)
        print "active_index: " + str(self.active_index)
        self.SetTaskActivity()

    def SetSubTreeActivity(self, set_activity_by_id, task_subtree):
        for task_id, task in task_subtree.iteritems():
            set_activity_by_id(task.id)
            if task.children:
                self.SetSubTreeActivity(set_activity_by_id, task.children)

    def SendSubTreeExecuteCommand(self, command, task_subtree):
        for task_id, task in task_subtree.iteritems():
            self.send_execute_command_callback(task.id, command)
            if task.children:
                self.SendSubTreeExecuteCommand(command, task.children)

    def GetTaskByIndex(self, task_index):
        task_id = self.view.TaskIndexToId(task_index)
        return self.model.GetTaskById(task_id)

    def HandleStart(self, event):
        print "HandleStart"
        active_task = self.GetTaskByIndex(self.active_index)
        if not active_task.children:
            self.send_execute_command_callback(active_task.id, 1)
        else:
            self.SendSubTreeExecuteCommand(1, active_task.children)

    def HandleStop(self, event):
        print "HandleStop"
        active_task = self.GetTaskByIndex(self.active_index)
        if not active_task.children:
            self.send_execute_command_callback(active_task.id, 0)
        else:
            self.SendSubTreeExecuteCommand(0, active_task.children)

    def TaskStatusChanged(self, task_status):
        self.view.SetTaskLoadById(task_status.id, task_status.load)
        self.view.SetTaskMemoryById(task_status.id, task_status.memory)
        self.view.SetTaskOutputById(task_status.id, task_status.stdout)

    def AddTaskEntriesDepthFirst(self, tasks, depth=0):
        for task in tasks.itervalues():
            if depth == 0 and task.parent:
                continue
            self.view.AddTask(task.id, task.config.name, depth)
            if task.children:
                self.AddTaskEntriesDepthFirst(task.children, depth+1)

    def TasksChanged(self, tasks):
        print "[TaskMinionController] TasksChanged"
        self.AddTaskEntriesDepthFirst(tasks)

    def SetModelTaskStatus(self, task_status):
        self.model.SetTaskStatus(task_status)

    def SetRequestRegisterCommandCallback(self, function):
        self.RequestRegisterCommand = function

    def ReceivedMasterProcessConfig(self):
        return self.received_master_process_config

    def SetMasterProcessConfig(self, process_task_list):
        print "[TaskMinionController::SetMasterProcessConfig] Setting master process config"
        self.received_master_process_config = True
        self.model.SetProcessTaskList(process_task_list)

    def RegisterCommands(self, process_task_list):
        print "TaskMinionController::RegisterCommands"
        registered_process_task_list = []
        for proc in process_task_list:
            proc.id = self.RequestRegisterCommand(proc)
            registered_process_task_list.append(proc)

        return registered_process_task_list

    def LoadYamlConfiguration(self, config_file):
        with open(config_file, 'r') as stream:
            yaml_process_config = yaml.load(stream)

        if yaml_process_config:
            return self.ConvertFromYamlProcessConfig(yaml_process_config)
        else:
            print "[TaskMinionModel::LoadYamlConfiguration] Empty YAML process configuration."
            print "Exiting..."
            sys.exit()

    def ConvertFromYamlProcessConfig(self, yaml_process_config):
        process_task_list = []
        for proc in yaml_process_config:
            name = proc
            command = ""
            group = ""
            dependencies = []
            for config in yaml_process_config[proc]:
                for param in config:
                    if param == "command":
                        command = config[param]
                    if param == "group":
                        group = config[param]
                    if param == "dependencies":
                        for dependency in config[param]:
                            dependencies.append(dependency)

            print "name: " + name
            print "command: " + command
            print "group: " + group
            print "dependencies: " + "[%s]" % ", ".join(map(str, dependencies))

            process_task = TaskConfig()
            process_task.name = name
            process_task.command = command
            process_task.group = group
            process_task.dependencies = dependencies

            process_task_list.append(process_task)

        return process_task_list

    def SetSendExecuteCommandCallback(self, callback):
        self.send_execute_command_callback = callback

    def Run(self):
        signal.signal(signal.SIGINT, signal_handler)

        while not self.ReceivedMasterProcessConfig():
            print "Waiting for process config message from master"
            time.sleep(0.2)

        # check if the master config was empty
        if not self.model.HasTasks():
            print "No processes registered with master"
            print "Loading configuration from yaml file"
            config_path = "/home/fla/task_master_ws/src/task_minion/cfg/process_config.yaml"
            process_task_list = self.LoadYamlConfiguration(config_path)
            registered_process_task_list = self.RegisterCommands(process_task_list)
            self.model.SetProcessTaskList(registered_process_task_list)

        self.view.root.mainloop()