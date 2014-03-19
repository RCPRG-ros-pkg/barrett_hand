#! /usr/bin/env python

import roslib; roslib.load_manifest('barrett_hand_controller')
import rospy
import sys
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *

def move_hand_client(prefix, f1, f2, f3, spread):
    rospy.wait_for_service('/' + prefix + '_hand/move_hand')
    try:
        move_hand = rospy.ServiceProxy('/' + prefix + '_hand/move_hand', BHMoveHand)
        resp1 = move_hand(f1*3.14, f2*3.14, f3*3.14, spread*3.14, 0.7, 0.7, 0.7, 0.7, 1000, 1000, 1000, 1000)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('move_hand_py', anonymous=True)

        a = []
        for arg in sys.argv:
            a.append(arg)

	if 6 == len(a):
            move_hand_client(a[1], float(a[2]), float(a[3]), float(a[4]), float(a[5]))
        else:
            print "Usage: %s prefix f1 f2 f3 spread"%a[0]

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

