# Copyright 2018 Massachusetts Institute of Technology

class ProcessStatus:
    def __init__(self):
        self.id = -1
        # self.status = ""
        self.load = 0
        self.memory = 0
        self.stdout = ""
        # self.message = ""

class ProcessCommand:
    def __init__(self):
        self.id = -1
        self.name = ""
        self.command = ""
        self.group = ""
        self.dependencies = []

class ProcessConfig:
    def __init__(self):
        self.commands = []

class TaskMinionModel:
    def __init__(self):
        self.yaml_process_config = {}  # raw yaml config file
        self.process_config = {}  # dictionary (indexed by process id) of registered ProcessCommands
        self.process_statuses = {}  # dictionary (indexed by process id) of ProcessStatuses

    def SetProcessConfig(self, process_config):
        if not process_config.commands:
            print "[TaskMinionModel::SetProcessConfig] Received empty process configuration"

        for proc in process_config.commands:
            self.SetProcessCommand(proc)

    def SetProcessCommand(self, process_command):
        if process_command.id in self.process_config:
            print "[TaskMinionModel::SetProcessCommand] Overwriting process id: " + string(process_command.id)

        self.process_config[process_command.id] = process_command

    def HasProcessConfig(self):
        if self.process_config:
            return True
        return False

    def ProcessStatusExists(self, process_id):
        if process_id in self.process_statuses:
            return True
        return False

    def SetProcessStatus(self, process_status):
        if not ProcessStatusExists(process_status.id):
            process_status[process_status.id] = ProcessStatus()

        self.process_status[process_status.id].load = process_status.load
        self.process_status[process_status.id].memory = process_status.memory
        self.process_status[process_status.id].stdout = self.process_status[process_status.id].stdout + process_status.stdout

    def GetProcessCount(self):
        return len(self.process_config)