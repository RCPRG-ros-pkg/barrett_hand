#! /usr/bin/env python

import roslib; roslib.load_manifest('barrett_hand_controller')
import rospy
import sys
from barrett_hand_controller.msg import *
from barrett_hand_controller_srvs.srv import *

def move_hand(prefix, spread, f1, f2, f3):

        global pub
	cmd = BHCmd()
	cmd.header.seq = 0
        cmd.header.frame_id = ""
        cmd.header.stamp = rospy.Time.now()
	cmd.cmd[0] = f1*3.14
	cmd.cmd[1] = f2*3.14
	cmd.cmd[2] = f3*3.14
	cmd.cmd[3] = spread*3.14
        pub.publish(cmd)
#	print cmd

def move_hand_client(prefix, f1, f2, f3, spread):
    rospy.wait_for_service('/' + prefix + '_hand/move_hand')
    try:
        move_hand = rospy.ServiceProxy('/' + prefix + '_hand/move_hand', BHMoveHand)
	cmd = BHMoveHand()
	cmd.f1 = f1
	cmd.f2 = f2
	cmd.f3 = f3
	cmd.sp = spread
	cmd.f1_speed = 0.7
	cmd.f2_speed = 0.7
	cmd.f3_speed = 0.7
	cmd.sp_speed = 0.7
	cmd.f1_torque = 1000
	cmd.f2_torque = 1000
	cmd.f3_torque = 1000
	cmd.sp_torque = 1000
        resp1 = move_hand(f1*3.14, f2*3.14, f3*3.14, spread*3.14, 0.7, 0.7, 0.7, 0.7, 1000, 1000, 1000, 1000)
#        resp1 = move_hand(cmd)
#        return resp1.result
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
#            pub = rospy.Publisher('/' + a[1] + '_hand/BHCmdx', BHCmd)
#            move_hand(float(a[2]), float(a[3]), float(a[4]), float(a[5]))
#            rospy.sleep(1)
#            move_hand(float(a[2]), float(a[3]), float(a[4]), float(a[5]))
        else:
            print "Usage: %s prefix f1 f2 f3 spread"%a[0]

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

