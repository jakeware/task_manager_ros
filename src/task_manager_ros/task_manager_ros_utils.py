# Copyright 2018 Massachusetts Institute of Technology

import rospy

from task_manager_ros.msg import *
from task_manager_ros.srv import *
from task_manager import task_manager_core

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
    task_config = task_manager_core.TaskConfig(task_config_msg.id)
    task_config.name = task_config_msg.name
    task_config.command = task_config_msg.command
    task_config.group = task_config_msg.group
    for dep in task_config_msg.dependencies:
        task_config.dependencies.append(dep)

    return task_config

def ConvertToRosTaskConfigList(task_config_list):
    task_config_list_msg = task_manager_ros.msg.TaskConfigList()
    task_config_list_msg.header.stamp = rospy.Time.now()
    for conf in task_config_list:
        task_config_msg = ConvertToRosTaskConfig(conf)
        task_config_list_msg.task_configs.append(task_config_msg)

    return task_config_list_msg

def ConvertFromRosTaskConfigList(task_config_list_msg):
    task_config_list = []
    for conf in task_config_list_msg.task_configs:
        task_config = ConvertFromRosTaskConfig(conf)
        task_config_list.append(task_config)

    return task_config_list

def ConvertToRosTaskInfo(task_info):
    task_info_msg = task_manager_ros.msg.TaskInfo()
    task_info_msg.id = task_info.id
    task_info_msg.status = task_info.status
    task_info_msg.load = task_info.load
    task_info_msg.memory = task_info.memory
    task_info_msg.stdout = task_info.stdout
    task_info_msg.stdout_delta = task_info.stdout_delta

    return task_info_msg

def ConvertFromRosTaskInfo(task_info_msg):
    task_info = task_manager_core.TaskInfo(task_info_msg.id)
    task_info.status = task_info_msg.status
    task_info.load = task_info_msg.load
    task_info.memory = task_info_msg.memory
    task_info.stdout = task_info_msg.stdout
    task_info.stdout_delta = task_info_msg.stdout_delta

    return task_info