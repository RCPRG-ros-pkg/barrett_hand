#! /usr/bin/env python

import roslib; roslib.load_manifest('barrett_hand_controller')
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

import barrett_hand_controller.msg


def move_hand(spread, f1, f2, f3):

    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/move_hand', barrett_hand_controller.msg.BHMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = barrett_hand_controller.msg.BHMoveGoal()
    
    goal.fingerVel = 1.2
    goal.spreadVel = 1.2
    goal.spread = spread*3.14
    goal.finger[0] = f1*3.14
    goal.finger[1] = f2*3.14
    goal.finger[2] = f3*3.14
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    client.wait_for_result()
    # Waits for the server to finish performing the action.
    rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('move_hand_py')

        a = []
        for arg in sys.argv:
            print arg
            a.append(arg)

        move_hand(float(a[1]), float(a[2]), float(a[3]), float(a[4]))
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
