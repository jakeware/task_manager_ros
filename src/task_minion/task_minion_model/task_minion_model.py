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
        self.process_statuses = {}  # dictionary (indexed by process id) of ProcessStatuses
        self.sorted_process_config = []  # ordered processes where each element is a [id, process_command] pair.  Groups are stored as [-1. name].
        self.process_status_callbacks = []
        self.process_command_callbacks = []

    def GetProcessGroups(self, process_config):
        process_groups = []

        # add all groups in process_config
        for proc in process_config.commands:
            if proc.group not in process_groups:
                process_groups.append(proc.group)

        print "[TaskMinionModel::GetProcessGroups] Process Groups:"
        print process_groups

        return process_groups

    def SetSortedProcessConfig(self, process_config, process_groups):
        self.sorted_process_config = []

        # add all processes in groups first
        for grp in process_groups:
            self.AddGroupCommand(grp, len(self.sorted_process_config))

            for proc in process_config.commands:
                if grp == proc.group:
                    self.AddProcessCommand(proc, len(self.sorted_process_config))

        # now add processes not in a group
        for proc in process_config.commands:
            if not proc.group:
                self.AddProcessCommand(proc, len(self.sorted_process_config))

        print "[TaskMinionModel::GetProcessOrder] Process Order:"
        print self.sorted_process_config

    def SetProcessConfig(self, process_config):
        if not process_config.commands:
            print "[TaskMinionModel::SetProcessConfig] Received empty process configuration"
            return

        print "[TaskMinionModel::SetProcessConfig] Loading new process configuration"
        process_groups = self.GetProcessGroups(process_config)
        self.SetSortedProcessConfig(process_config, process_groups)

    def AddGroupCommand(self, group_name, index):
        self.sorted_process_config.insert(index, [-1, group_name])

        for callback in self.process_command_callbacks:
            callback(-1, index, group_name)

    def AddProcessCommand(self, process_command, index):
        self.sorted_process_config.insert(index, [process_command.id, process_command])

        for callback in self.process_command_callbacks:
            callback(process_command.id, index, process_command.name)

    def HasProcessConfig(self):
        if self.sorted_process_config:
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

        for callback in self.process_status_callbacks:
            callback(process_status.id)

    def GetProcessCount(self):
        return len(self.sorted_process_config)

    def AddProcessCommandCallback(self, callback):
        self.process_command_callbacks.append(callback)

    def AddProcessStatusCallback(self, callback):
        self.process_status_callbacks.append(callback)