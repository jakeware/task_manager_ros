# Copyright 2018 Massachusetts Institute of Technology

import rospy
from std_msgs.msg import String

class TestProcessRos(object):
    def __init__(self, num):
        print "TestProcess" + num + "::Constructor"
        self.num = num
        self.count = 0
        self.param1 = rospy.get_param('~param1', 'blah')
        self.pub = rospy.Publisher('output_topic', String, queue_size=10)

    def Run(self):
        print "TestProcess" + self.num + "::Run"
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.PrintOutput()
            r.sleep()

    def PrintOutput(self):
        output_string = "TestProcess" + self.num + " Count:" + str(self.count) + " Param1:" + self.param1
        print output_string
        self.pub.publish(output_string)
        self.count = self.count + 1
