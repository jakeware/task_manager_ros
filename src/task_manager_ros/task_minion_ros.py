# Copyright 2018 Massachusetts Institute of Technology

import rospy

from task_manager_ros.msg import *
from task_manager_ros.srv import *
from task_manager_ros import task_manager_ros_utils
from task_manager import task_minion_controller

class TaskMinionRos(object):
    def __init__(self):
        print "TaskMinionRos::Constructor"
        self.task_config_path = rospy.get_param('~task_config_path')
        print self.task_config_path
        self.controller = task_minion_controller.TaskMinionController(self.task_config_path)
        self.controller.SetRequestRegisterTaskCallback(self.RequestRegisterTask)
        self.controller.SetPublishTaskCommandCallback(self.PublishTaskCommand)

    def RequestRegisterTask(self, task_config):
        task_config_msg = task_manager_ros_utils.ConvertToRosTaskConfig(task_config)
        rospy.wait_for_service('/task_master/register_task')
        try:
            register_task = rospy.ServiceProxy('/task_master/register_task', RegisterTask)
            res = register_task(task_config_msg)
            return res.id
        except rospy.ServiceException, e:
            print "RegisterTask service call failed: %s"%e

    def PublishTaskCommand(self, task_id, command):
        print "[TaskMinionRos::PublishTaskCommand] Called for id:" + str(task_id) + " with command:" + command
        task_command = task_manager_ros.msg.TaskCommand()
        task_command.id = task_id
        task_command.command = command

        self.task_command_publisher.publish(task_command)

    def TaskConfigListCallback(self, task_config_list_msg):
        # print "TaskMinionRos::TaskConfigListCallback"
        task_config_list = task_manager_ros_utils.ConvertFromRosTaskConfigList(task_config_list_msg)
        self.controller.PushMasterTaskConfigList(task_config_list)

    def TaskInfoListCallback(self, task_info_list_msg):
        # print "TaskMinionRos::TaskStatusCallback"
        task_info_list = task_manager_ros_utils.ConvertFromRosTaskInfoList(task_info_list_msg)
        for task_info in task_info_list:
            self.controller.PushTaskInfo(task_info)

    def Run(self):
        print "TaskMinionRos::Run"
        rospy.loginfo("Starting TaskMinionRos\n")
        rospy.Subscriber("/task_master/task_config_list", task_manager_ros.msg.TaskConfigList, self.TaskConfigListCallback)
        rospy.Subscriber("/task_master/task_info_list", task_manager_ros.msg.TaskInfoList, self.TaskInfoListCallback)
        self.task_command_publisher = rospy.Publisher('/task_master/task_command', task_manager_ros.msg.TaskCommand, queue_size=10)

        self.controller.Run()