# Copyright 2018 Massachusetts Institute of Technology

import rospy
from std_msgs.msg import String
from task_manager.msg import *
from task_manager.srv import *

class TaskMasterRos(object):
    def __init__(self):
        print "TaskMasterRos::Constructor"
        self.next_task_id = 0
        self.task_configs = {}

    def RegisterTaskCallback(self, req):
        task_id = self.next_task_id
        self.task_configs[req.task_config.id] = req.task_config
        print "[TaskMaster::RegisterTaskCallback] Registered task:" + req.task_config.name + " with id:" + str(self.next_task_id)

        self.next_task_id = self.next_task_id + 1
        return RegisterTaskResponse(task_id)

    def TaskCommandCallback(self, req):
        print "TaskMaster::TaskCommandCallback"

    def PublishTaskConfigList(self, event):
        task_config_list_msg = TaskConfigList()
        for conf in self.task_configs.itervalues():
            task_config_list_msg.task_configs.append(conf)

        self.task_config_list_pub.publish(task_config_list_msg)

    def Run(self):
        print "TaskMasterRos::Run"
        rospy.loginfo("Starting TaskMasterRos\n")
        self.task_info_pub = rospy.Publisher('~task_info', task_manager.msg.TaskInfo, queue_size=10)
        self.task_config_list_pub = rospy.Publisher('~task_config_list', task_manager.msg.TaskConfigList, queue_size=10)
        rospy.Subscriber("~task_command", task_manager.msg.TaskCommand, self.TaskCommandCallback)
        self.register_task_srv = rospy.Service('~register_task', RegisterTask, self.RegisterTaskCallback)

        rospy.Timer(rospy.Duration(0.1), self.PublishTaskConfigList)
        rospy.spin()