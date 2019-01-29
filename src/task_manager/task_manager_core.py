# Copyright 2018 Massachusetts Institute of Technology

class TaskInfo(object):
    def __init__(self, task_id=-1):
        self.id = task_id
        self.status = ""
        self.load = -1
        self.memory = -1
        self.stdout = ""
        self.stdout_delta = ""
        # self.message = ""

class TaskConfig(object):
    def __init__(self, task_id=-1):
        self.id = task_id
        self.name = ""
        self.command = ""  # command to be executed
        self.group = ""  # name of group containing this task
        self.dependencies = []  # list of dependent tasks

class Task(object):
    def __init__(self, task_id=-1):
        self.id = task_id
        self.parent = None  # parent task
        self.children = {}  # dictionary (indexed by task and group id) of child Tasks
        self.info = TaskInfo(task_id)
        self.config = TaskConfig(task_id)

    def AddChild(self, task):
        self.children[task.id] = task