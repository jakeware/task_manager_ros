#!/usr/bin/env python
# Copyright 2018 Massachusetts Institute of Technology

import rospy

from test_nodes import test_process_ros

if __name__ == '__main__':
    node_num = '3'
    rospy.init_node('test_node' + node_num)
    test_process = test_process_ros.TestProcessRos(node_num)
    test_process.Run()