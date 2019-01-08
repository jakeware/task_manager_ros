#!/usr/bin/env python
# Copyright 2018 Massachusetts Institute of Technology

import rospy
from task_minion_ros import *

if __name__ == '__main__':
    rospy.init_node("task_minion_ros_node")
    task_minion_ros = TaskMinionRos()
    task_minion_ros.Run()