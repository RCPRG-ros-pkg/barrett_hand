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
#calibration = 100
#initial_force = []
end = []

def moveToPos(f1, f2, f3, sp):

    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/move_hand', barrett_hand_controller.msg.BHMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = barrett_hand_controller.msg.BHMoveGoal()
    
    goal.fingerVel = 1.2
    goal.spreadVel = 1.2
    goal.spread = sp
    goal.finger[0] = f1
    goal.finger[1] = f2
    goal.finger[2] = f3
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    client.wait_for_result()
    # Waits for the server to finish performing the action.
    rospy.sleep(1.0)

def move(f1, f2, f3, s):
    global finger_vel_pub

    fingerVel = BHFingerVel()
    fingerVel.vel.append(f1)
    fingerVel.vel.append(f2)
    fingerVel.vel.append(f3)
    fingerVel.vel.append(s)
    finger_vel_pub.publish(fingerVel)

def callback(data):
    global calibration
    global initial_force
    global update
    global end

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
        
#    if calibration > 0:
#        initial_force[0] += mean[0]
#        initial_force[1] += mean[1]
#        initial_force[2] += mean[2]
#        calibration = calibration - 1
#    elif calibration == 0:
#        initial_force[0] = initial_force[0]/100
#        initial_force[1] = initial_force[1]/100
#        initial_force[2] = initial_force[2]/100
#        calibration = calibration - 1
#        print "calibrated %s: %s"%(sens, initial_force[sens])

    vel = []

    for sens in range(0, 3):
        diff = mean[sens];# - initial_force[sens]
        if diff<0:
            diff = 0.0
        val = (0.05 - diff)*5
        if (val<0.0) and (end[sens]==False):
            print "sensor %s: stop"%(sens)
            end[sens] = True

        if val<-1:
            val = -1
        elif val>1:
            val = 1

        if end[sens] == True:
           val = 0
        vel.append(val)

#    vel[2] = 0
    move(vel[0]*2,vel[1]*2,vel[2]*2,0.0)
                
if __name__ == '__main__':

    global initial_force
#    initial_force.append(0.0)
#    initial_force.append(0.0)
#    initial_force.append(0.0)
#    initial_force.append(0.0)
    end.append(False)
    end.append(False)
    end.append(False)
    end.append(False)
    rospy.init_node('hand_markers', anonymous=True)
    moveToPos(0,0,0, 0.0*3.14)
    finger_vel_pub = rospy.Publisher('barrett_hand_controller/finger_vel', BHFingerVel)
    rospy.Subscriber("/barrett_hand_controller/BHPressureState", BHPressureState, callback)
    rospy.spin()


