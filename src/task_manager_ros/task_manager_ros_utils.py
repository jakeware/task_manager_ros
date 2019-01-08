# Copyright 2018 Massachusetts Institute of Technology

import rospy

from task_manager_ros.msg import *
from task_manager_ros.srv import *
from task_manager import task_minion_model

def ConvertToRosTaskConfig(task_config):
    task_config_msg = task_manager_ros.msg.TaskConfig()
    task_config_msg.header.stamp = rospy.Time.now()
    task_config_msg.id = task_config.id
    task_config_msg.name = task_config.name
    task_config_msg.command = task_config.command
    task_config_msg.group = task_config.group
    task_config_msg.dependencies = task_config.dependencies

    return task_config_msg

def ConvertFromRosTaskConfig(task_config_msg):
    task_config = task_minion_model.TaskConfig(task_config_msg.id)
    task_config.name = task_config_msg.name
    task_config.command = task_config_msg.command
    task_config.group = task_config_msg.group
    for dep in task_config_msg.dependencies:
        task_config.dependencies.append(dep)

    return task_config

def ConvertFromRosTaskConfigList(task_config_list_msg):
    task_config_list = []
    for conf in task_config_list_msg.task_configs:
        task_config = ConvertFromRosTaskConfig(conf)
        task_config_list.append(task_config)

    return task_config_list

def ConvertFromRosTaskInfo(task_info_msg):
    task_info = task_minion_model.TaskInfo(task_info_msg.id)
    task_info.load = task_info_msg.load
    task_info.memory = task_info_msg.memory
    task_info.stdout_delta = task_info_msg.stdout

    return task_info