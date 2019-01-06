# Copyright 2018 Massachusetts Institute of Technology

import rospy
from std_msgs.msg import String
from task_manager.msg import *
from task_manager.srv import *

from task_minion.task_minion_model import *
from task_minion.task_minion_controller import *

class TaskMinionRos(object):
    def __init__(self):
        print "TaskMinionRos::Constructor"
        self.task_config_path = rospy.get_param('~task_config_path')
        print self.task_config_path
        self.controller = TaskMinionController(self.task_config_path)
        self.controller.SetRequestRegisterTaskCallback(self.RequestRegisterTask)
        self.controller.SetPublishTaskCommandCallback(self.PublishTaskCommand)

    def RequestRegisterTask(self, task_config):
        task_config_msg = self.ConvertToRosTaskConfig(task_config)
        rospy.wait_for_service('/task_master/register_task')
        try:
            register_task = rospy.ServiceProxy('/task_master/register_task', RegisterTask)
            res = register_task(task_config_msg)
            return res.id
        except rospy.ServiceException, e:
            print "RegisterTask service call failed: %s"%e

    def PublishTaskCommand(self, task_id, command):
        print "[TaskMinionRos::PublishTaskCommand] Called for id:" + str(task_id) + " with command:" + command
        task_command = task_manager.msg.TaskCommand()
        task_command.id = task_id
        task_command.command = command

        self.task_command_publisher.publish(task_command)

    def ConvertToRosTaskConfig(self, task_config):
        task_config_msg = task_manager.msg.TaskConfig()
        task_config_msg.header.stamp = rospy.Time.now()
        task_config_msg.id = task_config.id
        task_config_msg.name = task_config.name
        task_config_msg.command = task_config.command
        task_config_msg.group = task_config.group
        task_config_msg.dependencies = task_config.dependencies

        return task_config_msg

    def ConvertFromRosTaskConfig(self, task_config_msg):
        task_config = TaskConfig(task_config_msg.id)
        task_config.name = task_config_msg.name
        task_config.command = task_config_msg.command
        task_config.group = task_config_msg.group
        for dep in task_config_msg.dependencies:
            task_config.dependencies.append(dep)

        return task_config

    def ConvertFromRosTaskConfigList(self, task_config_list_msg):
        task_config_list = []
        for conf in task_config_list_msg.task_configs:
            task_config = self.ConvertFromRosTaskConfig(conf)
            task_config_list.append(task_config)

        return task_config_list

    def ConvertFromRosTaskInfo(self, task_info_msg):
        task_info = TaskInfo(task_info_msg.id)
        task_info.load = task_info_msg.load
        task_info.memory = task_info_msg.memory
        task_info.stdout_delta = task_info_msg.stdout

        return task_info

    def TaskConfigListCallback(self, task_config_list_msg):
        # print "TaskMinionRos::TaskConfigListCallback"
        if not self.controller.ReceivedMasterTaskConfigList():
            task_config_list = self.ConvertFromRosTaskConfigList(task_config_list_msg)
            self.controller.SetMasterTaskConfigList(task_config_list)

    def TaskInfoCallback(self, task_info_msg):
        # print "TaskMinionRos::TaskStatusCallback"
        task_info = self.ConvertFromRosProcessStatus(status_msg)
        self.controller.SetModelTaskInfo(task_info)

    def Run(self):
        print "TaskMinionRos::Run"
        rospy.loginfo("Starting TaskMinionRos\n")
        rospy.Subscriber("/task_master/task_config_list", task_manager.msg.TaskConfigList, self.TaskConfigListCallback)
        rospy.Subscriber("/task_master/task_info", task_manager.msg.TaskInfo, self.TaskInfoCallback)
        self.task_command_publisher = rospy.Publisher('/task_master/task_command', task_manager.msg.TaskCommand, queue_size=10)

        self.controller.Run()