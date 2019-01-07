# Copyright 2018 Massachusetts Institute of Technology

import rospy
import roslaunch
import rospkg
import Queue
from std_msgs.msg import String
from task_manager.msg import *
from task_manager.srv import *

class TaskMasterRos(object):
    def __init__(self):
        print "TaskMasterRos::Constructor"
        self.next_task_id = 0
        self.task_configs = {}
        self.task_command_queue = Queue.Queue()

    def RegisterTaskCallback(self, req):
        task_id = self.next_task_id
        self.task_configs[task_id] = req.task_config
        print "[TaskMaster::RegisterTaskCallback] Registered task:" + req.task_config.name + " with id:" + str(task_id)

        self.next_task_id = self.next_task_id + 1
        return RegisterTaskResponse(task_id)

    def TaskCommandCallback(self, task_command):
        print "[TaskMaster::TaskCommandCallback] Got command:" + task_command.command + " for id:" + str(task_command.id)
        self.task_command_queue.put(task_command)

    def PublishTaskConfigList(self, event):
        task_config_list_msg = TaskConfigList()
        for conf in self.task_configs.itervalues():
            task_config_list_msg.task_configs.append(conf)

        self.task_config_list_pub.publish(task_config_list_msg)

    def GetTaskConfigById(self, task_id):
        print self.task_configs
        return self.task_configs[task_id]

    def ExecuteTaskCommand(self, task_command):
        print "TaskMasterRos::ExecuteTaskCommand"
        if task_command.command == 'start':
            task_config = self.GetTaskConfigById(task_command.id)
            self.StartTask(task_config)

    def StartTask(self, task_config):
        print "TaskMasterRos::StartTaskCommand"
        split_command = task_config.command.split(' ')
        print split_command

        if (split_command[0] == 'roslaunch'):
            package = split_command[1]
            launch_file = split_command[2]
            self.StartLaunchFile(package, launch_file)

    def GetPackagePath(self, package):
        rospack = rospkg.RosPack()
        return rospack.get_path(package)

    def StartLaunchFile(self, package, launch_file):
        print "TaskMasterRos::StartLaunchFile"
        package_path = self.GetPackagePath(package)
        print package_path
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        # cli_args = ['/home/fla/task_master_ws/src/task_manager/launch/test_node1.launch','load_params:=false']
        # roslaunch_args = cli_args[1:]
        # roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        roslaunch_file = [package_path + '/launch/' + launch_file]
        # print roslaunch_file

        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()

    def Run(self):
        print "TaskMasterRos::Run"
        rospy.loginfo("Starting TaskMasterRos\n")
        self.task_info_pub = rospy.Publisher('~task_info', task_manager.msg.TaskInfo, queue_size=10)
        self.task_config_list_pub = rospy.Publisher('~task_config_list', task_manager.msg.TaskConfigList, queue_size=10)
        rospy.Subscriber("~task_command", task_manager.msg.TaskCommand, self.TaskCommandCallback)
        self.register_task_srv = rospy.Service('~register_task', RegisterTask, self.RegisterTaskCallback)

        rospy.Timer(rospy.Duration(0.1), self.PublishTaskConfigList)
        while not rospy.is_shutdown():
            while self.task_command_queue.qsize():
                try:
                    task_command = self.task_command_queue.get(0)
                    self.ExecuteTaskCommand(task_command)
                except Queue.Empty:
                    pass
            rospy.sleep(0.1)
