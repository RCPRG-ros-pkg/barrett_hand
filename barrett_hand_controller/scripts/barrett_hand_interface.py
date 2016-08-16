#!/usr/bin/env python

# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import tf

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from barrett_hand_controller_msgs.msg import *
from visualization_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from threading import Lock

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import geometry_msgs.msg

import PyKDL
import math
from numpy import *
import numpy as np

import copy

# reference frames:
# B - robot's base
# W - wrist
# E - gripper
# F - finger distal link
# T - tool

class BarrettHand:
    """
Class for BarrettHand robot.
"""

    def resetFingers(self):
        msg = std_msgs.msg.Empty()
        self.pub_reset.publish(msg)

    def getJointStatesByNames(self, dof_names):
        js = []
        for joint_name in dof_names:
            js.append( self.js_pos[joint_name] )
        return js

    def getLastJointState(self):
        self.joint_states_lock.acquire()
        js = self.js_pos_history[self.js_pos_history_idx]
        self.joint_states_lock.release()
        return js

    def getJointStateAtTime(self, time):
        self.joint_states_lock.acquire()
        hist_len = len(self.js_pos_history)
        for step in range(hist_len-1):
            h1_idx = (self.js_pos_history_idx - step - 1) % hist_len
            h2_idx = (self.js_pos_history_idx - step) % hist_len
            if self.js_pos_history[h1_idx] == None or self.js_pos_history[h2_idx] == None:
                self.joint_states_lock.release()
                return None

            time1 = self.js_pos_history[h1_idx][0]
            time2 = self.js_pos_history[h2_idx][0]
            if time > time1 and time <= time2:
                factor = (time - time1).to_sec() / (time2 - time1).to_sec()
                js_pos = {}
                for joint_name in self.js_pos_history[h1_idx][1]:
                    js_pos[joint_name] = self.js_pos_history[h1_idx][1][joint_name] * (1.0 - factor) + self.js_pos_history[h2_idx][1][joint_name] * factor
                self.joint_states_lock.release()
                return js_pos
        self.joint_states_lock.release()
        return None

    def jointStatesCallback(self, data):
        self.joint_states_lock.acquire()
        joint_idx = 0
        for joint_name in data.name:
            self.js_pos[joint_name] = data.position[joint_idx]
            joint_idx += 1

        self.js_pos_history_idx = (self.js_pos_history_idx + 1) % len(self.js_pos_history)
        self.js_pos_history[self.js_pos_history_idx] = (data.header.stamp, copy.copy(self.js_pos))
        self.joint_states_lock.release()

        if self.js_names_vector == None:
            js_names_vector = []
            self.js_inactive_names_vector = []
            for joint_name in data.name:
                if joint_name.startswith(self.prefix + '_Hand'):
                    self.js_inactive_names_vector.append(joint_name)
                else:
                    js_names_vector.append(joint_name)
            vector_len = len(js_names_vector)
            self.lim_lower = np.empty(vector_len)
            self.lim_lower_soft = np.empty(vector_len)
            self.lim_upper = np.empty(vector_len)
            self.lim_upper_soft = np.empty(vector_len)

            self.js_names_vector = js_names_vector

    def waitForInit(self):
        while not rospy.is_shutdown():
            can_break = True
            if self.js_names_vector == None:
                can_break = False
            if can_break:
                break
            rospy.sleep(0.1)

    def getJointStatesVector(self):
        q = np.empty(len(self.js_names_vector))
        q_idx = 0
        for joint_name in self.js_names_vector:
            q[q_idx] = self.js_pos[joint_name]
            q_idx += 1
        return q

    def getInactiveJointStatesVector(self):
        q = np.empty(len(self.js_inactive_names_vector))
        q_idx = 0
        for joint_name in self.js_inactive_names_vector:
            q[q_idx] = self.js_pos[joint_name]
            q_idx += 1
        return q

    def getJointStatesVectorNames(self):
        return self.js_names_vector

    def getInactiveJointStatesVectorNames(self):
        return self.js_inactive_names_vector

    def getJointLimitsVectors(self):
        return self.lim_lower, self.lim_upper

    def __init__(self, prefix):

        self.js_names_vector = None
        self.js_pos = {}
        self.js_pos_history = []
        for i in range(200):
            self.js_pos_history.append( None )
        self.js_pos_history_idx = 0

        # parameters
        self.prefix = prefix

#        self.last_contact_time = rospy.Time.now()

        self.joint_states_lock = Lock()

        # for score function
        self.failure_reason = "unknown"

        self.emergency_stop_active = False

        self.action_move_hand_client = actionlib.SimpleActionClient("/" + self.prefix + "_hand/move_hand", BHMoveAction)
        self.action_move_hand_client.wait_for_server()

        self.pub_reset = rospy.Publisher("/" + self.prefix + "_hand/reset_fingers", std_msgs.msg.Empty, queue_size=100)

        self.listener = tf.TransformListener();
        self.br = tf.TransformBroadcaster()

        rospy.sleep(1.0)
        joint_states_listener = rospy.Subscriber('/joint_states', JointState, self.jointStatesCallback)

    def moveHand(self, q, v, t, maxPressure, hold=False):
        action_goal = BHMoveGoal()
        action_goal.name = [self.prefix+"_HandFingerOneKnuckleTwoJoint", self.prefix+"_HandFingerTwoKnuckleTwoJoint", self.prefix+"_HandFingerThreeKnuckleTwoJoint", self.prefix+"_HandFingerOneKnuckleOneJoint"]
        action_goal.q = q
        action_goal.v = v
        action_goal.t = t
        action_goal.maxPressure = maxPressure
        if hold == True:
            action_goal.hold = 1
        else:
            action_goal.hold = 0
        self.action_move_hand_client.send_goal(action_goal)

    def waitForHand(self):
        self.action_move_hand_client.wait_for_result()
        return self.action_move_hand_client.get_result()

#    def hasContact(self, threshold, print_on_false=False):
#        if self.T_F_C != None:
#            return True
#        return False

    def updateTransformations(self):

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        self.T_E_F = pm.fromTf(pose)
        self.T_F_E = self.T_E_F.Inverse()

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerOneKnuckleThreeLink', rospy.Time(0))
        self.T_E_F13 = pm.fromTf(pose)
        self.T_F13_E = self.T_E_F13.Inverse()

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerTwoKnuckleThreeLink', rospy.Time(0))
        self.T_E_F23 = pm.fromTf(pose)
        self.T_F23_E = self.T_E_F23.Inverse()

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        self.T_E_F33 = pm.fromTf(pose)
        self.T_F33_E = self.T_E_F33.Inverse()

        pose = self.listener.lookupTransform(self.prefix+'_HandPalmLink', self.prefix+'_HandFingerOneKnuckleOneLink', rospy.Time(0))
        self.T_E_F11 = pm.fromTf(pose)

        pose = self.listener.lookupTransform(self.prefix+'_HandPalmLink', self.prefix+'_HandFingerTwoKnuckleOneLink', rospy.Time(0))
        self.T_E_F21 = pm.fromTf(pose)

        self.T_E_F31 = PyKDL.Frame()

