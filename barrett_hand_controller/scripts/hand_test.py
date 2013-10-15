#! /usr/bin/env python

import roslib; roslib.load_manifest('barrett_hand_controller')
import rospy

# Brings in the SimpleActionClient
import actionlib

import barrett_hand_controller.msg


def fibonacci_client():

    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/move_hand', barrett_hand_controller.msg.BHMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = barrett_hand_controller.msg.BHMoveGoal()
    
    goal.fingerVel = 0.2
    goal.spreadVel = 0.2
    goal.spread = 3.14
    goal.finger[0] = 1.2
    goal.finger[1] = 1.2
    goal.finger[2] = 1.2
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    client.wait_for_result()
    # Waits for the server to finish performing the action.
    rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        fibonacci_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
