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
        self.model.AddProcessStatusCallback(self.ProcessStatusChanged)
        self.model.AddProcessCommandCallback(self.ProcessCommandChanged)

        self.received_master_process_config = False  # have we received a process config from TaskMaster yet?
        self.active_index = 0

        # bind handlers to input keys
        self.root.bind('<Up>', self.HandleUp)
        self.root.bind('<Down>', self.HandleDown)
        self.root.bind('<Control-s>', self.HandleStart)
        self.root.bind('<Control-k>', self.HandleStop)

    def HandleUp(self, event):
        last_active_index = self.active_index
        self.active_index = max(self.active_index - 1, 0)
        print "active_index: " + str(self.active_index)

        if last_active_index != self.active_index:
            self.view.SetProcessInactive(last_active_index)
            self.view.SetProcessActive(self.active_index)

    def HandleDown(self, event):
        print "HandleDown"
        last_active_index = self.active_index
        self.active_index = min(self.active_index + 1, self.model.GetProcessCount() - 1)
        print "active_index: " + str(self.active_index)

        if last_active_index != self.active_index:
            self.view.SetProcessInactive(last_active_index)
            self.view.SetProcessActive(self.active_index)

    def HandleStart(self, event):
        print "HandleStart"

    def HandleStop(self, event):
        print "HandleStop"

    def ProcessStatusChanged(self, pid):
        print "[TaskMinionController] ProcessStatusChanged for pid:" + str(pid)

    def ProcessCommandChanged(self, pid, order, name):
        print "[TaskMinionController] ProcessCommandChanged for pid:" + str(pid)
        self.view.SetProcessEntry(pid, order, name)

    def SetProcessStatus(self, process_status):
        self.model.SetProcessStatus(process_status)

    def SetRequestRegisterCommandCallback(self, function):
        self.RequestRegisterCommand = function

    def ReceivedMasterProcessConfig(self):
        return self.received_master_process_config

    def SetMasterProcessConfig(self, process_config):
        print "[TaskMinionController::SetMasterProcessConfig] Setting master process config"
        self.received_master_process_config = True
        self.model.SetProcessConfig(process_config)

    def RegisterCommands(self, process_config):
        print "TaskMinionController::RegisterCommands"
        registered_process_config = ProcessConfig()
        for proc in process_config.commands:
            proc.id = self.RequestRegisterCommand(proc)
            registered_process_config.commands.append(proc)

        return registered_process_config

    def LoadYamlConfiguration(self, config_file):
        with open(config_file, 'r') as stream:
            yaml_process_config = yaml.load(stream)

        if yaml_process_config:
            process_config = self.ConvertFromYamlProcessConfig(yaml_process_config)
            return process_config
        else:
            print "[TaskMinionModel::LoadYamlConfiguration] Empty YAML process configuration."
            print "Exiting..."
            sys.exit()

    def ConvertFromYamlProcessConfig(self, yaml_process_config):
        process_config = ProcessConfig()
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

            process_command = ProcessCommand()
            process_command.name = name
            process_command.command = command
            process_command.group = group
            process_command.dependencies = dependencies

            process_config.commands.append(process_command)

        return process_config

    def Run(self):
        signal.signal(signal.SIGINT, signal_handler)

        while not self.ReceivedMasterProcessConfig():
            print "Waiting for process config message from master"
            time.sleep(0.2)

        # check if the master config was empty
        if not self.model.HasProcessConfig():
            print "No processes registered with master"
            print "Loading configuration from yaml file"
            config_path = "/home/fla/task_master_ws/src/task_master/cfg/process_config.yaml"
            process_config = self.LoadYamlConfiguration(config_path)
            registered_process_config = self.RegisterCommands(process_config)
            self.model.SetProcessConfig(registered_process_config)

        self.view.root.mainloop()