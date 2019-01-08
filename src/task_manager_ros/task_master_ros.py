# Copyright 2018 Massachusetts Institute of Technology

import rospy

from task_manager_ros.msg import *
from task_manager_ros.srv import *
from task_manager_ros import task_manager_ros_utils
from task_manager import task_master

class TaskMasterRos(object):
    def __init__(self):
        print "TaskMasterRos::Constructor"
        self.task_master = task_master.TaskMaster()
        self.task_master.SetPublishTaskConfigListCallback(self.PublishTaskConfigList)

    def RegisterTaskCallback(self, req):
        new_task_id = self.task_master.GetNewTaskId()
        req.task_config.id = new_task_id
        self.task_master.PushTaskConfig(req.task_config)
        print "[TaskMaster::RegisterTaskCallback] Registered task:" + req.task_config.name + " with id:" + str(new_task_id)
        return RegisterTaskResponse(new_task_id)

    def TaskCommandCallback(self, task_command):
        print "[TaskMaster::TaskCommandCallback] Got command:" + task_command.command + " for id:" + str(task_command.id)
        self.task_master.PushTaskCommand(task_command)

    def PublishTaskConfigList(self, task_config_list):
        task_config_list_msg = TaskConfigList()
        time_now = rospy.Time.now()
        task_config_list_msg.header.stamp = time_now
        for conf in task_config_list:
            task_config_msg = task_manager_ros_utils.ConvertToRosTaskConfig(conf)
            task_config_list_msg.task_configs.append(task_config_msg)
        self.task_config_list_pub.publish(task_config_list_msg)

    def Run(self):
        print "TaskMasterRos::Run"
        rospy.loginfo("Starting TaskMasterRos\n")
        self.task_info_pub = rospy.Publisher('~task_info', task_manager_ros.msg.TaskInfo, queue_size=10)
        self.task_config_list_pub = rospy.Publisher('~task_config_list', task_manager_ros.msg.TaskConfigList, queue_size=10)
        rospy.Subscriber("~task_command", task_manager_ros.msg.TaskCommand, self.TaskCommandCallback)
        self.register_task_srv = rospy.Service('~register_task', RegisterTask, self.RegisterTaskCallback)

        self.task_master.Run()