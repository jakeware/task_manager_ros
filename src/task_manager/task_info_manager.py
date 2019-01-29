# Copyright 2018 Massachusetts Institute of Technology

import Queue
import psutil
import signal
import threading
import os
import fcntl
import time
import sys

from task_manager import task_manager_core

class TaskInfoManager(object):
    def __init__(self):
        print "TaskInfoManager::Constructor"
        self.task_info_queues = {}  # dictionary of complete TaskInfo objects indexed by task_id
        self.task_info_lock = threading.Lock()

    def SetProcess(self, task_id, process):
        fcntl.fcntl(process.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        info_queue = Queue.Queue()
        info_thread = threading.Thread(target=self.UpdateTaskInfo, args=(task_id, process))
        info_thread.daemon = True  # thread dies with the program
        info_thread.start()
        self.SetTaskInfoQueue(task_id, info_queue)

    def TaskQueuesEmpty(self, task_id):
        return self.task_info_queues[task_id].empty()

    def RemoveTask(self, task_id):
        self.RemoveTaskInfoQueue(task_id)

    def SetTaskInfoQueue(self, task_id, task_info_queue):
        with self.task_info_lock:
            self.task_info_queues[task_id] = task_info_queue

    def RemoveTaskInfoQueue(self, task_id):
        with self.task_info_lock:
            del self.task_info_queues[task_id]

    def TaskInfoQueueExists(self, task_id):
        if task_id in self.task_info_queues:
            return True
        # print "[TaskInfoManager::TaskInfoQueueExists] Missing id:" + str(task_id)
        return False

    def GetTaskInfoQueueById(self, task_id):
        # print "TaskInfoManager::GetTaskInfoQueueById"
        with self.task_info_lock:
            if self.TaskInfoQueueExists(task_id):
                return self.task_info_queues[task_id]
            return None

    def GetTaskInfoQueuesKeys(self):
        with self.task_info_lock:
            return list(self.task_info_queues.keys())

    def GetTaskInfoById(self, task_id):
        # print "TaskInfoManager::GetTaskInfoById"
        task_info_queue = self.GetTaskInfoQueueById(task_id)
        if not task_info_queue:
            return None

        try:
            return task_info_queue.get_nowait()
        except Queue.Empty:
            pass

    def GetProcessLoad(self, pid):
        try:
            proc = psutil.Process(pid)
            return proc.cpu_percent(interval=0.1)
        except psutil.NoSuchProcess:
            print "[TaskInfoManager::GetProcessLoad] Task with PID:" + str(pid) + " no longer exists"
            return None

    def GetProcessMemory(self, pid):
        try:
            proc = psutil.Process(pid)
            return proc.memory_percent()
        except psutil.NoSuchProcess:
            print "[TaskInfoManager::GetProcessMemory] Task with PID:" + str(pid) + " no longer exists"
            return None

    def GetProcessIsRunning(self, pid):
        try:
            proc = psutil.Process(pid)
            return proc.is_running()
        except psutil.NoSuchProcess:
            print "[TaskInfoManager::GetProcessIsRunning] Task with PID:" + str(pid) + " no longer exists"
            return None

    def GetTaskOutput(self, process):
        # print "TaskInfoManager::PushTaskOutput"
        output = ''
        try:
            output += process.stdout.read()
        except:
            pass

        if output:
            return output
        else:
            return None

    def UpdateTaskInfo(self, task_id, process):
        # print "TaskInfoManager::UpdateTaskInfo"
        pid = process.pid
        while psutil.pid_exists(pid):
            task_info = task_manager_core.TaskInfo(task_id)

            # process load
            load = self.GetProcessLoad(pid)
            if load != None:
                task_info.load = load
            
            # process memory
            memory = self.GetProcessMemory(pid)
            if memory != None:
                task_info.memory = memory

            # process status
            is_running = self.GetProcessIsRunning(pid)
            if is_running != None:
                if is_running:
                    task_info.status = 'running'
                else:
                    task_info.status = 'stopped'

            # process output
            task_output = self.GetTaskOutput(process)
            if task_output:
                task_info.stdout_delta = task_output

            # add process info to queue
            task_info_queue = self.GetTaskInfoQueueById(task_id)
            task_info_queue.put(task_info)

            # check if process is still running
            if process.poll() != None:
                process.stdout.flush()
                sys.stdout.flush()

            time.sleep(0.1)
        process.stdout.close()
