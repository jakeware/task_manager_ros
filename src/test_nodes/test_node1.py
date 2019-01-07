#!/usr/bin/env python
# Copyright 2018 Massachusetts Institute of Technology

import rospy

class TestNode1(object):
    def __init__(self):
        print "TestNode1::Constructor"
        self.count = 0

    def Run(self):
        print "TestNode1::Run"
        rospy.Timer(rospy.Duration(0.5), self.PrintOutput)
        rospy.spin()

    def PrintOutput(self, event):
        print "[TestNode1::PrintOutput] Count:" + str(self.count)
        self.count = self.count + 1


if __name__ == '__main__':
    rospy.init_node("test_node1")
    test_node1 = TestNode1()
    test_node1.Run()