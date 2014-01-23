#! /usr/bin/env python

import roslib; roslib.load_manifest('barrett_hand_controller')
import rospy

# Brings in the SimpleActionClient
import actionlib

import sys
import rospy
import math

import std_msgs.msg
from barrett_hand_controller.msg import *
from barrett_hand_controller.srv import *
from geometry_msgs.msg import Point

update = 10
calibration = 100
initial_force = []
positions = []

def move(wait):
    global positions

    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/move_hand', barrett_hand_controller.msg.BHMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = barrett_hand_controller.msg.BHMoveGoal()
    
    goal.fingerVel = 1.2
    goal.spreadVel = 1.2
    goal.spread = 0
    goal.finger[0] = positions[0]
    goal.finger[1] = positions[1]
    goal.finger[2] = positions[2]
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    if wait == True:
        client.wait_for_result()
        # Waits for the server to finish performing the action.
        rospy.sleep(1.0)

def callback(data):
    global calibration
    global positions
    global initial_force
    global update

    frame_id = []
    frame_id.append("right_HandFingerOneKnuckleThreeLink")
    frame_id.append("right_HandFingerTwoKnuckleThreeLink")
    frame_id.append("right_HandFingerThreeKnuckleThreeLink")
    frame_id.append("right_HandPalmLink")

    mean = []
    mean.append(0.0)
    mean.append(0.0)
    mean.append(0.0)
    mean.append(0.0)
    sens = 0
    for sens in range(0, 3):
        mean[sens] = 0.0
        for i in range(0, 24):
            scale = 0.0
            if sens == 0:
                scale = data.finger1_tip[i]/256.0
            elif sens == 1:
                scale = data.finger2_tip[i]/256.0
            elif sens == 2:
                scale = data.finger3_tip[i]/256.0
            else:
                scale = data.palm_tip[i]/256.0
            mean[sens] = mean[sens] + scale

        mean[sens] = mean[sens]/24
        
#        else:
#            if update == 0:
#                update = 10
#                positions[sens] += (0.1 -(mean[sens] - initial_force[sens])*1)*0.1
#                if positions[sens]<0.0:
#                    positions[sens] = 0.0
#                elif positions[sens]>3.14:
#                    positions[sens] = 3.14
#                move()
#            if update == 0:
#                update = 200;
#                if positions[sens] == 0.0:
#                    positions[sens] = 2.0
#                else:
#                    positions[sens] = 0.0
#                move()

    if calibration > 0:
        initial_force[0] += mean[0]
        initial_force[1] += mean[1]
        initial_force[2] += mean[2]
        calibration = calibration - 1
    elif calibration == 0:
        initial_force[0] = initial_force[0]/100
        initial_force[1] = initial_force[1]/100
        initial_force[2] = initial_force[2]/100
        calibration = calibration - 1
#        print "calibrated %s: %s"%(sens, initial_force[sens])

    if update == 0:
        update = 5
        for sens in range(0, 3):
            diff = mean[sens] - initial_force[sens]
            if diff<0:
                diff = 0.0
            positions[sens] += (0.1 - diff)*0.5
            if positions[sens]<0.0:
                positions[sens] = 0.0
            elif positions[sens]>2.14:
                positions[sens] = 2.14
        move(False)
    update -= 1
                

def listener():
    rospy.init_node('hand_markers', anonymous=True)
    move(True)
    rospy.Subscriber("/barrett_hand_controller/BHPressureState", BHPressureState, callback)
    rospy.spin()

if __name__ == '__main__':
#    try:
#        # Initializes a rospy node so that the SimpleActionClient can
#        # publish and subscribe over ROS.
#        rospy.init_node('fibonacci_client_py')
#        fibonacci_client()
#    except rospy.ROSInterruptException:
#        print "program interrupted before completion"

    global initial_force
    global positions
    positions.append(0.0)
    positions.append(0.0)
    positions.append(0.0)
    positions.append(0.0)
    initial_force.append(0.0)
    initial_force.append(0.0)
    initial_force.append(0.0)
    initial_force.append(0.0)
    listener()


