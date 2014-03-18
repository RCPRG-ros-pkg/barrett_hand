#! /usr/bin/env python

import roslib; roslib.load_manifest('barrett_hand_controller')
import rospy
import sys
#import barrett_hand_controller.msg
from barrett_hand_controller.msg import *

def move_hand(spread, f1, f2, f3):

        global pub
	cmd = BHCmd()
	cmd.header.seq = 0
        cmd.header.frame_id = ""
        cmd.header.stamp = rospy.Time.now()
	cmd.cmd[0] = spread*3.14
	cmd.cmd[1] = f1*3.14
	cmd.cmd[2] = f2*3.14
	cmd.cmd[3] = f3*3.14
        pub.publish(cmd)
	print cmd
	#print "published: %s %s %s %s"%(cmd.cmd[0], cmd.cmd[1], cmd.cmd[2], cmd.cmd[3])

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('move_hand_py', anonymous=True)

        pub = rospy.Publisher('/barrett_hand_controller/BHCmdx', BHCmd)

        a = []
        for arg in sys.argv:
            print arg
            a.append(arg)

        move_hand(float(a[1]), float(a[2]), float(a[3]), float(a[4]))
        rospy.sleep(1)
        move_hand(float(a[1]), float(a[2]), float(a[3]), float(a[4]))
#        rospy.sleep(1)

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

