# Copyright 2018 Massachusetts Institute of Technology

class TaskStatus:
    def __init__(self):
        self.id = -1
        # self.status = ""
        self.load = 0
        self.memory = 0
        self.stdout = ""
        # self.message = ""

class Task:
    def __init__(self):
        self.id = -1
        self.name = ""
        self.command = ""
        self.group = ""
        self.dependencies = []
        self.children = {}

class TaskMinionModel:
    def __init__(self):
        self.task_statuses = {}  # dictionary (indexed by process and group id) of TaskStatuses
        self.task_tree = {}  # dictionary (indexed by process and group id) of Tasks
        self.task_status_callbacks = []
        self.process_task_list_callbacks = []

    def FindTaskByNameInTree(self, task_name, task_subtree):
        for task_id, task in task_subtree.iteritems():
            if task.name == task_name:
                return task
            if task.children:
                subtask = self.FindTaskByNameInTree(task_name, task.children)
                if subtask:
                    return subtask
        return None

    def GetTaskTree(self, process_task_list):
        self.task_tree = {}
        group_id = -2

        # add all tasks to task_tree (should be no groups in task list)
        for proc in process_task_list:
            # add task to root of tree if not in a group
            if not proc.group:
                self.task_tree[proc.id] = proc
                continue

            # search for group in tree
            group_task = self.FindTaskByNameInTree(proc.group, self.task_tree)

            # if group already exists in tree, add process to group
            if group_task:
                group_task.children[proc.id] = proc
            # otherwise, add new group to tree with process as child
            else:
                new_group_task = Task()
                new_group_task.id = group_id
                new_group_task.name = proc.group
                new_group_task.children[proc.id] = proc
                self.task_tree[group_id] = new_group_task
                group_id = group_id - 1

        print "[TaskMinionModel::GetProcessGroups] Task Tree:"
        self.PrintTaskTree(self.task_tree)

    def PrintTaskTree(self, task_subtree, depth=0):
        for task_id, task in task_subtree.iteritems():
            print str(depth) + ":" + task.name
            if task.children:
                self.PrintTaskTree(task.children, depth + 1)

    def SetProcessTaskList(self, process_task_list):
        if not process_task_list:
            print "[TaskMinionModel::SetProcessTaskList] Received empty process task list"
            return

        print "[TaskMinionModel::SetProcessTaskList] Loading new process task list"
        self.GetTaskTree(process_task_list)

        for callback in self.process_task_list_callbacks:
            callback(self.task_tree)

    def HasTaskTree(self):
        if self.task_tree:
            return True
        return False

    def TaskStatusExists(self, task_id):
        if task_id in self.task_statuses:
            return True
        return False

    def SetTaskStatus(self, process_status):
        if not TaskStatusExists(process_status.id):
            task_status[process_status.id] = TaskStatus()

        self.task_status[process_status.id].load = process_status.load
        self.task_status[process_status.id].memory = process_status.memory
        self.task_status[process_status.id].stdout = self.task_status[process_status.id].stdout + process_status.stdout

        for callback in self.task_status_callbacks:
            callback(process_status.id)

    def AddProcessConfigCallback(self, callback):
        self.process_task_list_callbacks.append(callback)

    def AddTaskStatusCallback(self, callback):
        self.task_status_callbacks.append(callback)