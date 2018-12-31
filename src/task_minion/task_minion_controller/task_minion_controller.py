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
    def __init__(self, model):
        self.root = Tk()
        self.view = TaskMinionView(self.root)
        self.model = model
        self.active_pid = 0

        # bind handlers to input keys
        self.root.bind('<Up>', self.HandleUp)
        self.root.bind('<Down>', self.HandleDown)
        self.root.bind('<Control-s>', self.HandleStart)
        self.root.bind('<Control-k>', self.HandleStop)

    def HandleUp(self, event):
        last_active_pid = self.active_pid
        self.active_pid = max(self.active_pid - 1, 0)
        print "active_pid: " + str(self.active_pid)

        if last_active_pid != self.active_pid:
            self.view.SetProcessInactive(last_active_pid)
            self.view.SetProcessActive(self.active_pid)

    def SetRequestRegisterCommandCallback(self, function):
        self.RequestRegisterCommand = function

    def HandleDown(self, event):
        print "HandleDown"
        last_active_pid = self.active_pid
        self.active_pid = min(self.active_pid + 1, self.model.GetProcessCount() - 1)
        print "active_pid: " + str(self.active_pid)

        if last_active_pid != self.active_pid:
            self.view.SetProcessInactive(last_active_pid)
            self.view.SetProcessActive(self.active_pid)

    def HandleStart(self, event):
        print "HandleStart"

    def HandleStop(self, event):
        print "HandleStop"

    def RegisterCommands(self, process_config):
        print "TaskMinionController::RegisterCommands"
        registered_process_config = ProcessConfig()
        for proc in process_config.commands:
            proc.id = self.RequestRegisterCommand(proc)
            registered_process_config.commands.append(proc)

        return registered_process_config

    def LoadYamlConfiguration(self, config_file):
        with open(config_file, 'r') as stream:
            self.yaml_process_config = yaml.load(stream)

        if self.yaml_process_config:
            process_config = self.ConvertFromYamlProcessConfig(self.yaml_process_config)
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

        while not self.model.ReceivedMasterProcessConfig():
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

        # add test processes to view
        for pid in range(self.model.GetProcessCount()):
            self.view.AddProcessEntry(pid)

        self.view.root.mainloop()