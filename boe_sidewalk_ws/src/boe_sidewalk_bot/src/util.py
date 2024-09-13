#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Empty

class Utility:
    def __init__(self):
        self.shutdownSub = rospy.Subscriber('/system/shutdown', Empty, self.shutdownCallback)
        self.rebootSub = rospy.Subscriber('/system/reboot', Empty, self.rebootCallback)
        self.rate = rospy.Rate(1)

    def shutdownCallback(self, msg):
        # print("System shutdown now...")
        os.system('shutdown now -h')

    def rebootCallback(self, msg):
        # print("System reboot now...")
        os.system('systemctl reboot -i')

if __name__ == '__main__':
    rospy.init_node('utility_node')
    utility = Utility()
    rospy.spin()