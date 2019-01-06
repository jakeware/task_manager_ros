#!/usr/bin/env python
# Copyright 2018 Massachusetts Institute of Technology

import rospy
from task_master_ros import *

if __name__ == '__main__':
    rospy.init_node("task_master_ros_node")
    task_master_ros = TaskMasterRos()
    task_master_ros.Run()