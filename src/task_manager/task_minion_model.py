# Copyright 2018 Massachusetts Institute of Technology

class TaskInfo(object):
    def __init__(self, task_id=-1):
        self.id = task_id
        self.status = ""
        self.load = 0
        self.memory = 0
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

class TaskMinionModel(object):
    def __init__(self):
        self.tasks = {}  # dictionary (indexed by task and group id) of all Tasks

    def FindTaskByName(self, task_name):
        for task in self.tasks.itervalues():
            if task.config.name == task_name:
                return task
        return None

    def GetTaskById(self, task_id):
        if self.TaskExists(task_id):
            return self.tasks[task_id]
        return None

    def AddTaskFromId(self, task_id):
        self.tasks[task_id] = Task(task_id)
        return self.tasks[task_id]

    def AddTaskFromConfig(self, task_config):
        task = self.AddTaskFromId(task_config.id)
        task.config = task_config
        return task

    def GetTaskInfoById(self, task_id):
        task = self.GetTaskById(task_id)
        if task:
            return task.info
        return None

    def GetTaskConfigById(self, task_id):
        task = self.GetTaskById(task_id)
        if task:
            return task.config
        return None

    def GetTasksFromTaskConfigList(self, task_config_list):
        self.task_tree = {}
        group_id = -2

        # add all tasks to task_tree (should be no groups in input task_config_list)
        for conf in task_config_list:
            # all tasks get added to root of dictionary
            task = self.AddTaskFromConfig(conf)

            # add group if it doesn't exist
            if conf.group:
                group_task = self.FindTaskByName(conf.group)
                # if group already exists in tree, set parent and add task to group
                if group_task:
                    task.parent = group_task
                    group_task.AddChild(task)
                # otherwise, add new group to tree with task as child
                else:
                    new_group_task = self.AddTaskFromId(group_id)
                    group_id = group_id - 1
                    new_group_task.config.name = conf.group
                    task.parent = new_group_task
                    new_group_task.AddChild(task)

        print "[TaskMinionModel::GetTasksFromTaskConfigList] Task Tree:"
        self.PrintTasks()

    def PrintTasks(self):
        print "Number of Tasks: " + str(len(self.tasks)) + "\n"
        for task in self.tasks.itervalues():
            print "ID: " + str(task.config.id)
            print "Name: " + task.config.name
            if task.config.group:
                print "Group: " + task.config.group
            if task.parent:
                print "Parent: {" + str(task.parent.id) + ", " + task.parent.config.name + "}"
            print "Children:"
            for child_task in task.children.itervalues():
                print "Child: {" + str(child_task.id) + ", " + child_task.config.name + "}"
            print "\n"

    def SetTaskConfigList(self, task_config_list):
        if not task_config_list:
            print "[TaskMinionModel::SetTaskConfigList] Received empty task config list"
            return

        print "[TaskMinionModel::SetTaskConfigList] Loading new task config list"
        self.GetTasksFromTaskConfigList(task_config_list)

        self.task_config_list_changed(self.tasks)

    def GetTaskConfigList(self):
        task_config_list = []
        for task in self.tasks.itervalues():
            task_config_list.append(task.config)
        return task_config_list

    def HasTasks(self):
        if self.tasks:
            return True
        return False

    def TaskExists(self, task_id):
        if task_id in self.tasks:
            return True
        print "[TaskMinionModel::TaskExists] Missing id:" + str(task_id)
        return False

    def SetTaskInfo(self, task_info):
        # print "TaskMinionModel::SetTaskInfo"
        task = self.GetTaskById(task_info.id)
        task.info.id = task_info.id
        task.info.status = task_info.status
        task.info.load = task_info.load
        task.info.memory = task_info.memory
        task.info.stdout_delta = task_info.stdout_delta
        task.info.stdout = task.info.stdout + task_info.stdout_delta
        self.task_info_changed(task.info)
        self.UpdateTaskInfo(task.parent)

    def UpdateTaskInfo(self, task):
        if not task:
            return

        load_total = 0
        memory_total = 0
        for child_task in task.children.itervalues():
            load_total = load_total + child_task.info.load
            memory_total = memory_total + child_task.info.memory
        task.info.load = load_total
        task.info.memory = memory_total
        self.task_info_changed(task.info)

        self.UpdateTaskInfo(task.parent)

    def SetTaskConfigListChangedCallback(self, callback):
        self.task_config_list_changed = callback

    def SetTaskInfoChangedCallback(self, callback):
        self.task_info_changed = callback