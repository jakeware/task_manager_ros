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
from task_manager import task_manager_core

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
        self.task_config_list_queue = Queue.Queue()
        self.task_config_path = task_config_path

        self.cooldown_time = 6.0  # time to wait before you can interact with a task again [s]
        self.received_master_task_config_list = False  # have we received a task config list from TaskMaster yet?
        self.cursor_index = 0
        self.active_indices = []  # list of row indices for all tasks associated with current cursor index
        self.selected_indices = []  # list of selected row indices in the task list gui panel
        self.last_action_times = {}  # dictionary (indexed by task_id) of times since action was last taken for debouncing

        # bind handlers to input keys
        self.root.bind('<Up>', self.HandleUp)
        self.root.bind('<Down>', self.HandleDown)
        self.root.bind('<Control-s>', self.HandleStart)
        self.root.bind('<Control-k>', self.HandleStop)
        self.root.bind('<Control-A>', self.HandleSelectAll)
        self.root.bind('<Control-a>', self.HandleSelect)

    def HandleSelectAll(self, event):
        print "HandleSelectAll"
        if set(range(self.view.GetTaskEntryCount())).issubset(self.selected_indices):
            for ind in range(self.view.GetTaskEntryCount()):
                self.DeselectIndex(ind)
        else:
            for ind in range(self.view.GetTaskEntryCount()):
                self.SelectIndex(ind)

        self.UpdateTaskActivity()
        self.UpdateTaskSelection()

    def HandleSelect(self, event):
        print "HandleSelect"
        if set(self.active_indices).issubset(self.selected_indices):
            for ind in self.active_indices:
                self.DeselectIndex(ind)
        else:
            for ind in self.active_indices:
                self.SelectIndex(ind)

        self.UpdateTaskActivity()
        self.UpdateTaskSelection()

    def UpdateTaskSelection(self):
        for ind in range(self.view.GetTaskEntryCount()):
            if ind in self.active_indices:
                continue

            selected_task = self.GetTaskByIndex(ind)
            if ind in self.selected_indices:
                self.view.SelectTaskById(selected_task.id)
            else:
                self.view.DeselectTaskById(selected_task.id)

    def SelectIndex(self, index):
        task = self.GetTaskByIndex(index)
        new_indices = [index]
        self.GetSubTreeIndices(new_indices, task.children)

        for ind in new_indices:
            if ind not in self.selected_indices:
                self.selected_indices.append(ind)

    def DeselectIndex(self, index):
        task = self.GetTaskByIndex(index)

        new_indices = [index]
        self.GetSubTreeIndices(new_indices, task.children)

        for ind in new_indices:
            if ind in self.selected_indices:
                self.selected_indices.remove(ind)

        parent_task = task.parent
        while parent_task:
            parent_index = self.view.TaskIdToIndex(parent_task.id)
            if parent_index in self.selected_indices:
                self.selected_indices.remove(parent_index)
            parent_task = parent_task.parent

    def DecrementCursorIndex(self):
        self.cursor_index = max(self.cursor_index - 1, 0)

    def IncrementCursorIndex(self):
        self.cursor_index = min(self.cursor_index + 1, self.view.GetTaskEntryCount() - 1)

    def HandleUp(self, event):
        self.DecrementCursorIndex()
        print "cursor_index: " + str(self.cursor_index)
        self.UpdateTaskActivity()
        self.UpdateTaskSelection()

    def HandleDown(self, event):
        self.IncrementCursorIndex()
        print "cursor_index: " + str(self.cursor_index)
        self.UpdateTaskActivity()
        self.UpdateTaskSelection()

    def UpdateTaskActivity(self):
        for ind in self.active_indices:
            task = self.GetTaskByIndex(ind)
            self.view.SetTaskInactiveById(task.id)

        cursor_task = self.GetTaskByIndex(self.cursor_index)
        self.active_indices = [self.cursor_index]
        self.GetSubTreeIndices(self.active_indices, cursor_task.children)

        for ind in self.active_indices:
            task = self.GetTaskByIndex(ind)
            self.view.SetTaskActiveById(task.id)
        self.view.SetTaskAndOutputActiveById(cursor_task.id)

    def GetSubTreeIndices(self, indices, task_subtree):
        for task in task_subtree.itervalues():
            indices.append(self.view.TaskIdToIndex(task.id))
            if task.children:
                self.GetSubTreeIndices(indices, task.children)

    def GetTaskByIndex(self, task_index):
        task_id = self.view.TaskIndexToId(task_index)
        return self.model.GetTaskById(task_id)

    def SetActionTime(self, task_id):
        self.last_action_times[task_id] = time.time()

    def TaskHasCooledDown(self, task_id):
        if task_id not in self.last_action_times:
            return True
        else:
            if self.last_action_times[task_id] < (time.time() - self.cooldown_time):
                return True
            else:
                return False

    def HandleStart(self, event):
        active_and_selected_indices = self.active_indices + list(set(self.selected_indices) - set(self.active_indices))
        for ind in active_and_selected_indices:
            task = self.GetTaskByIndex(ind)
            if task.id < 0:
                continue
            
            if not self.TaskHasCooledDown(task.id):
                print "[TaskMinionController] Task with id:" + str(task.id) + " still cooling down"
                continue
            
            self.SetActionTime(task.id)
            self.publish_task_command(task.id, 'start')

    def HandleStop(self, event):
        active_and_selected_indices = self.active_indices + list(set(self.selected_indices) - set(self.active_indices))
        for ind in active_and_selected_indices:
            task = self.GetTaskByIndex(ind)
            if task.id < 0:
                continue

            if not self.TaskHasCooledDown(task.id):
                print "[TaskMinionController] Task with id:" + str(task.id) + " still cooling down"
                continue

            self.SetActionTime(task.id)
            self.publish_task_command(task.id, 'stop')

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

    def PushTaskInfo(self, task_info):
        self.task_info_queue.put(task_info)

    def SetRequestRegisterTaskCallback(self, function):
        self.RequestRegisterTask = function

    def ReceivedMasterTaskConfigList(self):
        return self.received_master_task_config_list

    def PushMasterTaskConfigList(self, task_config_list):
        if not self.ReceivedMasterTaskConfigList():
            print "TaskMinionController::PushMasterTaskConfigList"
            self.received_master_task_config_list = True
            self.task_config_list_queue.put(task_config_list)
            self.UpdateTaskConfigList()

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

            task_config = task_manager_core.TaskConfig()
            task_config.name = name
            task_config.command = command
            task_config.group = group
            task_config.dependencies = dependencies

            task_config_list.append(task_config)

        return task_config_list

    def SetPublishTaskCommandCallback(self, callback):
        self.publish_task_command = callback

    def UpdateTaskInfo(self):
        while self.task_info_queue.qsize():
            try:
                task_info = self.task_info_queue.get(0)
                self.model.SetTaskInfo(task_info)
            except Queue.Empty:
                pass

    def UpdateTaskConfigList(self):
        while self.task_config_list_queue.qsize():
            try:
                task_config_list = self.task_config_list_queue.get(0)
                self.model.SetTaskConfigList(task_config_list)
            except Queue.Empty:
                pass

    def Update(self):
        self.UpdateTaskInfo()
        self.UpdateTaskConfigList()

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