# Copyright 2018 Massachusetts Institute of Technology

import rospy

class TestProcessRos(object):
    def __init__(self, num):
        print "TestProcess" + str(num) + "::Constructor"
        self.num = int(num)
        self.count = 0

    def Run(self):
        print "TestProcess" + str(self.num) + "::Run"
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.PrintOutput()
            r.sleep()

    def PrintOutput(self):
        print "TestProcess" + str(self.num) + " Count:" + str(self.count)
        self.count = self.count + 1