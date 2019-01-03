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

    def FindTaskByName(self, task_name, task_subtree):
        for task_id, task in task_subtree.iteritems():
            if task.name == task_name:
                return task
            if task.children:
                subtask = self.FindTaskByName(task_name, task.children)
                if subtask:
                    return subtask
        return None

    def FindTaskById(self, requested_task_id, task_subtree):
        for task_id, task in task_subtree.iteritems():
            if task.id == requested_task_id:
                return task
            if task.children:
                subtask = self.FindTaskById(requested_task_id, task.children)
                if subtask:
                    return subtask
        return None

    def GetTaskById(self, task_id):
        return self.FindTaskById(task_id, self.task_tree)

    def GetTaskStatusById(self, task_id):
        if task_id in self.task_statuses:
            return self.task_statuses[task_id]
        else:
            print "[TaskMinionModel::GetTaskStatusById] Missing id:" + str(task_id)

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
            group_task = self.FindTaskByName(proc.group, self.task_tree)

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
            print str(depth) + ", " + str(task_id) + ", " + task.name
            if task.children:
                self.PrintTaskTree(task.children, depth + 1)

    def SetProcessTaskList(self, process_task_list):
        if not process_task_list:
            print "[TaskMinionModel::SetProcessTaskList] Received empty process task list"
            return

        print "[TaskMinionModel::SetProcessTaskList] Loading new process task list"
        self.GetTaskTree(process_task_list)

        self.process_task_list_callback(self.task_tree)

    def HasTaskTree(self):
        if self.task_tree:
            return True
        return False

    def TaskStatusExists(self, task_id):
        if task_id in self.task_statuses:
            return True
        return False

    def SetTaskStatus(self, task_status):
        if not self.TaskStatusExists(task_status.id):
            self.task_statuses[task_status.id] = TaskStatus()

        self.task_statuses[task_status.id].id = task_status.id
        self.task_statuses[task_status.id].load = task_status.load
        self.task_statuses[task_status.id].memory = task_status.memory
        self.task_statuses[task_status.id].stdout = self.task_statuses[task_status.id].stdout + task_status.stdout

        self.task_status_callback(task_status.id)

    def SetProcessConfigCallback(self, callback):
        self.process_task_list_callback = callback

    def SetTaskStatusCallback(self, callback):
        self.task_status_callback = callback