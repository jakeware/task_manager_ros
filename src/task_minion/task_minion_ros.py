# Copyright 2018 Massachusetts Institute of Technology

import rospy
from std_msgs.msg import String
from task_master.msg import *
from task_master.srv import *

from task_minion_model import *
from task_minion_controller import *

class TaskMinionRos(object):
    def __init__(self):
        print "TaskMinionRos::Constructor"
        self.controller = TaskMinionController()
        self.controller.SetRequestRegisterCommandCallback(self.RequestRegisterCommand)
        self.controller.SetSendExecuteCommandCallback(self.SendExecuteCommand)

    def RequestRegisterCommand(self, process_command):
        command_msg = self.ConvertToRosProcessCommand(process_command)
        rospy.wait_for_service('/task_master/register_command')
        try:
            register_command = rospy.ServiceProxy('/task_master/register_command', RegisterCommand)
            res = register_command(command_msg)
            return res.id
        except rospy.ServiceException, e:
            print "RegisterCommand service call failed: %s"%e

    def SendExecuteCommand(self, process_id, command):
        print "[TaskMinionRos::SendExecuteCommand] Called for id:" + str(process_id) + " with command:" + str(command)
        execute_command = task_master.msg.ExecuteCommand()
        execute_command.id = process_id
        execute_command.command = command

        self.execute_command_publisher.publish(execute_command)

    def ConvertToRosProcessCommand(self, process_task):
        command_msg = task_master.msg.ProcessCommand()
        command_msg.header.stamp = rospy.Time.now()
        command_msg.id = process_task.id
        command_msg.name = process_task.name
        command_msg.command = process_task.command
        command_msg.group = process_task.group
        command_msg.dependencies = process_task.dependencies

        return command_msg

    def ConvertFromRosProcessCommand(self, command_msg):
        process_task = TaskConfig(command_msg.id)
        process_task.name = command_msg.name
        process_task.command = command_msg.command
        process_task.group = command_msg.group
        for dep in command_msg.dependencies:
            process_task.dependencies.append(dep)

        return process_task

    def ConvertFromRosProcessConfig(self, config_msg):
        process_task_list = []
        for proc in config_msg.commands:
            process_task = self.ConvertFromRosProcessCommand(proc)
            process_task_list.append(process_task)

        return process_task_list

    def ConvertFromRosProcessStatus(self, status_msg):
        task_status = TaskStatus(status_msg.id)
        task_status.load = status_msg.load
        task_status.memory = status_msg.memory
        task_status.stdout_delta = status_msg.stdout

        return task_status

    def ProcessConfigCallback(self, config_msg):
        print "TaskMinionRos::ProcessConfigCallback"
        if not self.controller.ReceivedMasterProcessConfig():
            process_task_list = self.ConvertFromRosProcessConfig(config_msg)
            self.controller.SetMasterProcessConfig(process_task_list)

    def ProcessStatusCallback(self, status_msg):
        print "TaskMinionRos::TaskStatusCallback"
        task_status = self.ConvertFromRosProcessStatus(status_msg)
        self.controller.SetModelTaskStatus(task_status)

    def Run(self):
        print "TaskMinionRos::Run"
        rospy.loginfo("Starting TaskMinionRos\n")
        rospy.Subscriber("/task_master/process_config", task_master.msg.ProcessConfig, self.ProcessConfigCallback)
        rospy.Subscriber("/task_master/process_status", task_master.msg.ProcessStatus, self.ProcessStatusCallback)
        self.execute_command_publisher = rospy.Publisher('/task_master/execute_command', task_master.msg.ExecuteCommand, queue_size=10)

        self.controller.Run()