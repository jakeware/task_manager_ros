#!/usr/bin/env python
# Copyright 2018 Massachusetts Institute of Technology

import rospy
from std_msgs.msg import String
from task_master.msg import *
from task_master.srv import *

from task_minion_controller.task_minion_controller import *
from task_minion_model.task_minion_model import *

class TaskMinion:
    def __init__(self):
        print "TaskMinion::Constructor"
        self.controller = TaskMinionController()
        self.controller.SetRequestRegisterCommandCallback(self.RequestRegisterCommand)

    def RequestRegisterCommand(self, process_command):
        command_msg = self.ConvertToRosProcessCommand(process_command)
        rospy.wait_for_service('/task_master/register_command')
        try:
            register_command = rospy.ServiceProxy('/task_master/register_command', RegisterCommand)
            res = register_command(command_msg)
            return res.id
        except rospy.ServiceException, e:
            print "RegisterCommand service call failed: %s"%e

    def ConvertToRosProcessCommand(self, process_command):
        command_msg = task_master.msg.ProcessCommand()
        command_msg.header.stamp = rospy.Time.now()
        command_msg.id = process_command.id
        command_msg.name = process_command.name
        command_msg.command = process_command.command
        command_msg.group = process_command.group
        command_msg.dependencies = process_command.dependencies

        return command_msg

    def ConvertFromRosProcessCommand(self, command_msg):
        process_command = ProcessCommand()
        process_command.id = command_msg.id
        process_command.name = command_msg.name
        process_command.command = command_msg.command
        process_command.group = command_msg.group
        for dep in command_msg.dependencies:
            process_command.dependencies.append(dep)

        return process_command

    def ConvertFromRosProcessConfig(self, config_msg):
        process_config = ProcessConfig()
        for proc in config_msg.commands:
            process_command = self.ConvertFromRosProcessCommand(proc)
            process_config.commands.append(process_command)
        return process_config

    def ConvertFromRosProcessStatus(self, status_msg):
        process_status = task_minion_model.task_minion_model.ProcessStatus()
        process_status.id = status_msg.id
        process_status.load = status_msg.load
        process_status.memory = status_msg.memory
        process_status.stdout = status_msg.stdout

        return process_status

    def ProcessConfigCallback(self, config_msg):
        print "TaskMinion::ProcessConfigCallback"
        if not self.controller.ReceivedMasterProcessConfig():
            process_config = self.ConvertFromRosProcessConfig(config_msg)
            self.controller.SetMasterProcessConfig(process_config)

    def ProcessStatusCallback(self, status_msg):
        print "TaskMinion::ProcessStatusCallback"
        process_status = self.ConvertFromRosProcessStatus(status_msg)
        self.controller.SetProcessStatus(process_status)

    def Run(self):
        print "TaskMinion::Run"
        rospy.loginfo("Starting TaskMinion\n")
        rospy.Subscriber("/task_master/process_config", task_master.msg.ProcessConfig, self.ProcessConfigCallback)
        rospy.Subscriber("/task_master/process_status", task_master.msg.ProcessStatus, self.ProcessStatusCallback)

        self.controller.Run()

if __name__ == '__main__':
    print "check"
    rospy.init_node("task_minion")
    task_minion = TaskMinion()
    task_minion.Run()