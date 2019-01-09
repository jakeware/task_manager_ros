# Copyright 2018 Massachusetts Institute of Technology

import Queue
import psutil
import signal
import threading

from task_manager import task_minion_model

class TaskInfoManager(object):
    def __init__(self):
        print "TaskInfoManager::Constructor"
        self.task_output_queues = {}  # dictionary of task output (strings) queues indexed by task_id
        self.task_stats_queues = {}  # dictionary of partial TaskInfo (no output) indexed by task_id
        self.task_info_queues = {}  # dictionary of complete TaskInfo objects indexed by task_id
        self.task_output_lock = threading.Lock()
        self.task_stats_lock = threading.Lock()
        self.task_info_lock = threading.Lock()

        info_thread = threading.Thread(target=self.UpdateTaskInfo)
        info_thread.start()

    def AddProcess(self, task_id, process):
        # add task stats stuff
        task_stats_queue = Queue.Queue()
        stats_thread = threading.Thread(target=self.UpdateTaskStats, args=(task_id, process, task_stats_queue))
        stats_thread.daemon = True  # thread dies with the program
        stats_thread.start()
        self.AddTaskStatsQueue(task_id, task_stats_queue)

        # add task output stuff
        output_queue = Queue.Queue()
        output_thread = threading.Thread(target=self.UpdateTaskOutput, args=(process.stdout, output_queue))
        output_thread.daemon = True  # thread dies with the program
        output_thread.start()
        self.AddTaskOutputQueue(task_id, output_queue)

        # add queue for complete task info
        info_queue = Queue.Queue()
        self.AddTaskInfoQueue(task_id, info_queue)

    def AddTaskOutputQueue(self, task_id, task_output_queue):
        # print "TaskInfoManager::AddTaskOutputQueue"
        with self.task_output_lock:
            self.task_output_queues[task_id] = task_output_queue

    def AddTaskInfoQueue(self, task_id, task_info_queue):
        # print "TaskInfoManager::AddTaskInfoQueue"
        with self.task_info_lock:
            self.task_info_queues[task_id] = task_info_queue

    def AddTaskStatsQueue(self, task_id, task_stats_queue):
        # print "TaskInfoManager::AddTaskStatsQueue"
        with self.task_stats_lock:
            self.task_stats_queues[task_id] = task_stats_queue

    def TaskOutputQueueExists(self, task_id):
        if task_id in self.task_output_queues:
            return True
        print "[TaskInfoManager::TaskOutputQueueExists] Missing id:" + str(task_id)
        return False

    def TaskInfoQueueExists(self, task_id):
        if task_id in self.task_info_queues:
            return True
        print "[TaskInfoManager::TaskInfoQueueExists] Missing id:" + str(task_id)
        return False

    def TaskStatsQueueExists(self, task_id):
        if task_id in self.task_stats_queues:
            return True
        print "[TaskInfoManager::TaskStatsQueueExists] Missing id:" + str(task_id)
        return False

    def GetTaskOutputQueueById(self, task_id):
        # print "TaskInfoManager::GetTaskOutputQueueById"
        with self.task_output_lock:
            if self.TaskOutputQueueExists(task_id):
                return self.task_output_queues[task_id]
            return None

    def GetTaskInfoQueueById(self, task_id):
        # print "TaskInfoManager::GetTaskInfoQueueById"
        with self.task_info_lock:
            if self.TaskInfoQueueExists(task_id):
                return self.task_info_queues[task_id]
            return None

    def GetTaskStatsQueueById(self, task_id):
        # print "TaskInfoManager::GetTaskStatsQueueById"
        with self.task_stats_lock:
            if self.TaskStatsQueueExists(task_id):
                return self.task_stats_queues[task_id]
            return None

    def GetTaskOutputById(self, task_id):
        task_output_queue = self.GetTaskOutputQueueById(task_id)
        if not task_output_queue:
            return None

        try:
            return task_output_queue.get_nowait() # or q.get(timeout=.1)
        except Queue.Empty:
            print "No output for process with task id:" + str(task_id)

        return None

    def GetTaskStatsById(self, task_id):
        task_stats_queue = self.GetTaskStatsQueueById(task_id)
        try:
            return task_stats_queue.get_nowait()
        except Queue.Empty:
            pass

    def GetTaskInfoById(self, task_id):
        # print "TaskInfoManager::GetTaskInfoById"
        task_info_queue = self.GetTaskInfoQueueById(task_id)
        try:
            return task_info_queue.get_nowait()
        except Queue.Empty:
            pass

    def UpdateTaskOutput(self, output, task_output_queue):
        # print "TaskInfoManager::PushTaskOutput"
        for line in iter(output.readline, b''):
            task_output_queue.put(line)
        output.close()

    def UpdateTaskStats(self, task_id, process, task_stats_queue):
        proc = psutil.Process(process.pid)

        while True:
            task_info = task_minion_model.TaskInfo(task_id)
            cpu_percent = proc.cpu_percent(interval=1)
            # print "cpu_percent: " + str(cpu_percent)
            task_info.load = cpu_percent

            memory_percent = proc.memory_percent()
            # print "memory_percent: " + str(memory_percent)
            task_info.memory = memory_percent

            is_running = proc.is_running()
            if is_running:
                task_info.status = 'running'
            else:
                task_info.status = 'stopped'
            # print "status: " + task_info.status

            task_stats_queue.put(task_info)

    def UpdateTaskInfo(self):
        # print "TaskInfoManager::UpdateTaskInfo"

        while True:
            task_id_list = []
            with self.task_info_lock:
                task_id_list = list(self.task_info_queues.keys())
            for task_id in task_id_list:
                task_stats = self.GetTaskStatsById(task_id)
                if not task_stats:
                    continue

                task_output = self.GetTaskOutputById(task_id)
                if task_output:
                    task_stats.stdout_delta = task_output                    
                task_info_queue = self.GetTaskInfoQueueById(task_id)
                task_info_queue.put(task_stats)