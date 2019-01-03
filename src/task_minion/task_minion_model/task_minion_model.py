# Copyright 2018 Massachusetts Institute of Technology

class TaskStatus(object):
    def __init__(self, task_id=-1):
        self.id = task_id
        # self.status = ""
        self.load = 0
        self.memory = 0
        self.stdout = ""
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
        self.children = {}  # dictionary (indexed by process and group id) of all Tasks
        self.status = TaskStatus(task_id)
        self.config = TaskConfig(task_id)

    def AddChild(self, task):
        self.children[task.id] = task

class TaskMinionModel(object):
    def __init__(self):
        self.tasks = {}  # dictionary (indexed by process and group id) of all Tasks

    def FindTaskByName(self, task_name):
        for task in self.tasks.itervalues():
            if task.config.name == task_name:
                return task
        return None

    def GetTaskById(self, task_id):
        if TaskExists(task_id):
            return self.tasks[task_id]
        return None

    def AddTaskFromId(self, task_id):
        self.tasks[task_id] = Task(task_id)
        return self.tasks[task_id]

    def AddTaskFromConfig(self, task_config):
        task = self.AddTaskFromId(task_config.id)
        task.config = task_config
        return task

    def GetTaskStatusById(self, task_id):
        task = self.GetTaskById(task_id)
        if task:
            return task.status
        return None

    def GetTaskConfigById(self, task_id):
        task = self.GetTaskById(task_id)
        if task:
            return task.config
        return None

    def GetTasksFromProcessConfigList(self, process_task_list):
        self.task_tree = {}
        group_id = -2

        # add all tasks to task_tree (should be no groups in input process_task_list)
        for conf in process_task_list:
            # all processes get added to root of dictionary
            task = self.AddTaskFromConfig(conf)

            # add group if it doesn't exist
            if conf.group:
                group_task = self.FindTaskByName(conf.group)
                # if group already exists in tree, set parent and add process to group
                if group_task:
                    print "Found Group: " + group_task.config.name
                    task.parent = group_task
                    group_task.AddChild(task)
                # otherwise, add new group to tree with process as child
                else:
                    print "Missing Group: " + conf.group
                    print "Adding Group ID: " + str(group_id)
                    new_group_task = self.AddTaskFromId(group_id)
                    if not new_group_task:
                        print "Got empty group task!!!!"
                    print "Added Group ID: " + str(new_group_task.id)
                    if new_group_task.id in self.tasks:
                        print "  In Tasks"
                    else:
                        print "  Not In Tasks"
                    group_id = group_id - 1
                    print "Reprint Group ID: " + str(new_group_task.id)
                    new_group_task.config.name = conf.group
                    print "Check Name: " + self.tasks[new_group_task.id].config.name
                    task.parent = new_group_task
                    print "Check Parent: " + self.tasks[task.id].parent.config.name
                    new_group_task.AddChild(task)
                    print "Check Children:"
                    for child_task in new_group_task.children.itervalues():
                        "Child: { " + str(child_task.id) + ", " + child_task.config.name + " }"
                        print "\n"

        print "[TaskMinionModel::GetProcessGroups] Task Tree:"
        self.PrintTasks()

    def PrintTasks(self):
        print "Number of Tasks: " + str(len(self.tasks))
        for task in self.tasks.itervalues():
            print "ID: " + str(task.config.id)
            print "Name: " + task.config.name
            if task.config.group:
                print "Group: " + task.config.group
            if task.parent:
                print "Parent: { " + str(task.parent.id) + ", " + task.parent.config.name + " }"
            print "Children:"
            for child_task in task.children.itervalues():
                "Child: { " + str(child_task.id) + ", " + child_task.config.name + " }"
            print "\n"

    def SetProcessTaskList(self, process_task_list):
        if not process_task_list:
            print "[TaskMinionModel::SetProcessTaskList] Received empty process task list"
            return

        print "[TaskMinionModel::SetProcessTaskList] Loading new process task list"
        self.GetTasksFromProcessConfigList(process_task_list)

        self.process_task_list_callback(self.task_tree)

    def HasTasks(self):
        if self.tasks:
            return True
        return False

    def TaskExists(self, task_id):
        if task_id in self.tasks:
            return True
        print "[TaskMinionModel::GetTaskById] Missing id:" + str(task_id)
        return False

    def SetTaskStatus(self, task_status):
        task = self.GetTaskById(task_status.id)
        task.status.id = task_status.id
        task.status.load = task_status.load
        task.status.memory = task_status.memory
        task.status.stdout = task.status.stdout + task_status.stdout
        self.task_status_callback(task_status.id)
        self.UpdateTaskStatusById(task.parent)

    def UpdateTaskStatusById(self, task_id):
        if task.parent:
            return

        task = self.GetTaskById(task_id)
        load_total = 0
        memory_total = 0
        for child_task in child_task.children.itervalues():
            load_total = load_total + task.load
            memory_total = memory_total + task.memory
        self.task_status_callback(task.id)
        self.UpdateTaskStatusById(task.parent)

    def SetProcessConfigCallback(self, callback):
        self.process_task_list_callback = callback

    def SetTaskStatusCallback(self, callback):
        self.task_status_callback = callback