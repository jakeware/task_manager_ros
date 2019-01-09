# Copyright 2018 Massachusetts Institute of Technology

import sys
import os
import yaml
import time
import signal
import Queue
import threading

from Tkinter import *
from ScrolledText import ScrolledText

from task_manager import task_minion_view
from task_manager import task_minion_model

def signal_handler(sig, frame):
        print('[TaskMinion] Caught SIGINT. Exiting...')
        sys.exit(0)

class TaskMinionController(object):
    def __init__(self, task_config_path):
        self.root = Tk()
        self.view = task_minion_view.TaskMinionView(self.root)
        self.model = task_minion_model.TaskMinionModel()
        self.model.SetTaskInfoChangedCallback(self.TaskInfoChanged)
        self.model.SetTaskConfigListChangedCallback(self.TasksChanged)
        self.task_info_queue = Queue.Queue()
        self.task_config_path = task_config_path

        self.received_master_task_config_list = False  # have we received a task config list from TaskMaster yet?
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
        for task in task_subtree.itervalues():
            set_activity_by_id(task.id)
            if task.children:
                self.SetSubTreeActivity(set_activity_by_id, task.children)

    def PublishSubTreeTaskCommand(self, command, task_subtree):
        for task in task_subtree.itervalues():
            self.publish_task_command(task.id, command)
            if task.children:
                self.SendSubTreeExecuteCommand(command, task.children)

    def GetTaskByIndex(self, task_index):
        task_id = self.view.TaskIndexToId(task_index)
        return self.model.GetTaskById(task_id)

    def HandleStart(self, event):
        print "HandleStart"
        active_task = self.GetTaskByIndex(self.active_index)
        if not active_task.children:
            self.publish_task_command(active_task.id, 'start')
        else:
            self.PublishSubTreeTaskCommand('start', active_task.children)

    def HandleStop(self, event):
        print "HandleStop"
        active_task = self.GetTaskByIndex(self.active_index)
        if not active_task.children:
            self.publish_task_command(active_task.id, 'stop')
        else:
            self.PublishSubTreeTaskCommand('stop', active_task.children)

    def TaskInfoChanged(self, task_info):
        self.view.SetTaskStatusById(task_info.id, task_info.status)
        self.view.SetTaskLoadById(task_info.id, task_info.load)
        self.view.SetTaskMemoryById(task_info.id, task_info.memory)
        self.view.SetTaskOutputDeltaById(task_info.id, task_info.stdout_delta)
        task_info.stdout_delta = ""

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

    def PushModelTaskInfo(self, task_info):
        self.task_info_queue.put(task_info)

    def SetRequestRegisterTaskCallback(self, function):
        self.RequestRegisterTask = function

    def ReceivedMasterTaskConfigList(self):
        return self.received_master_task_config_list

    def SetMasterTaskConfigList(self, task_config_list):
        print "[TaskMinionController::SetMasterTaskConfigList] Setting master task config"
        self.received_master_task_config_list = True
        self.model.SetTaskConfigList(task_config_list)

    def RegisterTasks(self, task_config_list):
        print "TaskMinionController::RegisterTasks"
        registered_task_config_list = []
        for config in task_config_list:
            config.id = self.RequestRegisterTask(config)
            registered_task_config_list.append(config)

        return registered_task_config_list

    def LoadYamlConfiguration(self, config_file):
        with open(config_file, 'r') as stream:
            yaml_task_config = yaml.load(stream)

        if yaml_task_config:
            return self.ConvertFromYamlTaskConfig(yaml_task_config)
        else:
            print "[TaskMinionModel::LoadYamlConfiguration] Empty YAML task configuration."
            print "Exiting..."
            sys.exit()

    def ConvertFromYamlTaskConfig(self, yaml_task_config):
        task_config_list = []
        for task_name in yaml_task_config:
            name = task_name
            command = ""
            group = ""
            dependencies = []
            for config in yaml_task_config[task_name]:
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

            task_config = task_minion_model.TaskConfig()
            task_config.name = name
            task_config.command = command
            task_config.group = group
            task_config.dependencies = dependencies

            task_config_list.append(task_config)

        return task_config_list

    def SetPublishTaskCommandCallback(self, callback):
        self.publish_task_command = callback

    def Update(self):
        while self.task_info_queue.qsize(  ):
            try:
                task_info = self.task_info_queue.get(0)
                self.model.SetTaskInfo(task_info)
            except Queue.Empty:
                pass

        self.root.after(100, self.Update)

    def Run(self):
        signal.signal(signal.SIGINT, signal_handler)

        while not self.ReceivedMasterTaskConfigList():
            print "Waiting for task config message from master"
            time.sleep(0.2)

        # check if the master config was empty
        if not self.model.HasTasks():
            print "No tasks registered with master"
            print "Loading configuration from path: " + self.task_config_path
            task_config_list = self.LoadYamlConfiguration(self.task_config_path)
            registered_task_config_list = self.RegisterTasks(task_config_list)
            self.model.SetTaskConfigList(registered_task_config_list)

        self.root.after(0, self.Update())
        self.root.mainloop()