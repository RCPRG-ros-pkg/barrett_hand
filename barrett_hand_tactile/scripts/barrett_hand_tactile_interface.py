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

class BarrettHandTactileInterface:
    """
Class for BarrettHand robot.
"""

    def getPressureSensorsInfo(self):
        self.is_updated = False
        self.pressure_info_tmp = None
        def tactileInfoCallback(msg):
            if not self.is_updated:
                self.pressure_info_tmp = msg
                self.is_updated = True
        rospy.Subscriber('/' + self.prefix + '_hand/tactile_info_out', barrett_hand_controller_msgs.msg.BHPressureInfo, tactileInfoCallback)
        rospy.sleep(1.0)
        return self.pressure_info_tmp

    def calibrateTactileSensors(self):
        msg = std_msgs.msg.Empty()
        self.pub_calibrate.publish(msg)

    def setMedianFilter(self, samples):
        # TODO
        print "setMedianFilter is not implemented"
#        try:
#            if not hasattr(self, 'set_median_filter') or self.set_median_filter == None:
#                rospy.wait_for_service('/' + self.prefix + '_hand/set_median_filter')
#                self.set_median_filter = rospy.ServiceProxy('/' + self.prefix + '_hand/set_median_filter', BHSetMedianFilter)
#            resp1 = self.set_median_filter(samples)
#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

    def tactileCallback(self, data):
        self.tactile_lock.acquire()
        self.tactile_data_index += 1
        if self.tactile_data_index >= self.tactile_data_len:
            self.tactile_data_index = 0
        tact_buf_idx = self.tactile_data_index

        # time, finger, max value, contact center
        self.tactile_data[tact_buf_idx][0] = copy.copy(data.header.stamp)
        self.tactile_data[tact_buf_idx][1] = copy.copy(data.finger1_tip)
        self.tactile_data[tact_buf_idx][2] = copy.copy(data.finger2_tip)
        self.tactile_data[tact_buf_idx][3] = copy.copy(data.finger3_tip)
        self.tactile_data[tact_buf_idx][4] = copy.copy(data.palm_tip)
        self.tactile_lock.release()

    def getTactileData(self):
        self.tactile_lock.acquire()
        index = copy.copy(self.tactile_data_index)
        self.tactile_lock.release()
        return copy.deepcopy(self.tactile_data[index])

    def __init__(self, prefix):

        # parameters
        self.prefix = prefix

        self.last_contact_time = rospy.Time.now()

        self.tactile_lock = Lock()

        # for tactile sync
        self.tactile_data = []
        self.tactile_data_len = 120
        self.tactile_data_index = 0
        for i in range(0, self.tactile_data_len):
            self.tactile_data.append( [rospy.Time.now(), [], [], [], []] )
            for j in range(24):
                self.tactile_data[-1][1].append(0)
                self.tactile_data[-1][2].append(0)
                self.tactile_data[-1][3].append(0)
                self.tactile_data[-1][4].append(0)

        # for score function
        self.pub_calibrate = rospy.Publisher("/" + self.prefix + "_hand/calibrate_tactile_sensors", std_msgs.msg.Empty, queue_size=100)

        self.listener = tf.TransformListener();
        self.br = tf.TransformBroadcaster()

        rospy.sleep(1.0)
        
        self.pressure_info = self.getPressureSensorsInfo()
        self.pressure_frames = []
        self.pressure_cells_size = []
        for i in range(0, 24):
            center = PyKDL.Vector(self.pressure_info.sensor[0].center[i].x, self.pressure_info.sensor[0].center[i].y, self.pressure_info.sensor[0].center[i].z)
            halfside1 = PyKDL.Vector(self.pressure_info.sensor[0].halfside1[i].x, self.pressure_info.sensor[0].halfside1[i].y, self.pressure_info.sensor[0].halfside1[i].z)
            halfside2 = PyKDL.Vector(self.pressure_info.sensor[0].halfside2[i].x, self.pressure_info.sensor[0].halfside2[i].y, self.pressure_info.sensor[0].halfside2[i].z)
            self.pressure_cells_size.append( (halfside1.Norm()*2.0, halfside2.Norm()*2.0) )
            halfside1.Normalize()
            halfside2.Normalize()
            norm = halfside1*halfside2
            norm.Normalize()
            self.pressure_frames.append( PyKDL.Frame(PyKDL.Rotation(halfside1, halfside2, norm), center) )

        self.palm_pressure_frames = []
        self.palm_pressure_cells_size = []
        for i in range(0, 24):
            center = PyKDL.Vector(self.pressure_info.sensor[3].center[i].x, self.pressure_info.sensor[3].center[i].y, self.pressure_info.sensor[3].center[i].z)
            halfside1 = PyKDL.Vector(self.pressure_info.sensor[3].halfside1[i].x, self.pressure_info.sensor[3].halfside1[i].y, self.pressure_info.sensor[3].halfside1[i].z)
            halfside2 = PyKDL.Vector(self.pressure_info.sensor[3].halfside2[i].x, self.pressure_info.sensor[3].halfside2[i].y, self.pressure_info.sensor[3].halfside2[i].z)
            self.palm_pressure_cells_size.append( (halfside1.Norm()*2.0, halfside2.Norm()*2.0) )
            halfside1.Normalize()
            halfside2.Normalize()
            norm = halfside1*halfside2
            norm.Normalize()
            self.palm_pressure_frames.append( PyKDL.Frame(PyKDL.Rotation(halfside1, halfside2, norm), center) )

        rospy.Subscriber('/' + self.prefix + '_hand/BHPressureState', BHPressureState, self.tactileCallback)

    def getContactPointsInFrame(self, threshold, frame_name):
        self.tactile_lock.acquire()
        latest_index = copy.copy(self.tactile_data_index)
        self.tactile_lock.release()

        tactile_frames_names = [
        '/'+self.prefix+'_HandFingerOneKnuckleThreeLink',
        '/'+self.prefix+'_HandFingerTwoKnuckleThreeLink',
        '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink',
        '/'+self.prefix+'_HandPalmLink']
        contacts = []
        forces = []

        pressure_frames = [self.pressure_frames, self.pressure_frames, self.pressure_frames, self.palm_pressure_frames]
        for tact_idx in range(len(tactile_frames_names)):
            tact_name = tactile_frames_names[tact_idx]
            for buf_prev_idx in range(20, self.tactile_data_len-2):
                buf_idx = latest_index - buf_prev_idx
#                buf_idx_prev = buf_idx - 1
                if buf_idx < 0:
                    buf_idx += self.tactile_data_len
#                if buf_idx_prev < 0:
#                    buf_idx_prev += self.tactile_data_len

                time = self.tactile_data[buf_idx][0]
                tactile_data = self.tactile_data[buf_idx][tact_idx+1]

                if self.listener.canTransform('torso_base', tact_name, time) and self.listener.canTransform('torso_base', frame_name, time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', tact_name, time))
                    T_B_R = pm.fromTf(self.listener.lookupTransform('torso_base', frame_name, time))
                    T_R_B = T_B_R.Inverse()
                    for i in range(0, len(pressure_frames[tact_idx])):
#                        print "i"
                        neighbourhood_ok = True
                        # check the time neighbourhood
                        for buf_neigh in range(-19, 19):
                            buf_neigh_idx = buf_idx+buf_neigh
                            if buf_neigh_idx < 0:
                                buf_neigh_idx += self.tactile_data_len
                            elif buf_neigh_idx >= self.tactile_data_len:
                                buf_neigh_idx -= self.tactile_data_len
#                            print buf_neigh_idx
#                            print self.tactile_data[0][1]
                            if self.tactile_data[buf_neigh_idx][tact_idx+1][i] < threshold:
#                            if self.tactile_data[0][1][0] < threshold:
                                neighbourhood_ok = False
                                break
                        if neighbourhood_ok:#tactile_data[i] > threshold:
#                            contacts.append( T_R_B * T_B_F * pressure_frames[tact_idx][i] * PyKDL.Vector() )
                            h1 = self.pressure_info.sensor[tact_idx].halfside1[i]
                            h2 = self.pressure_info.sensor[tact_idx].halfside2[i]
                            contacts.append( (T_R_B * T_B_F * pressure_frames[tact_idx][i], PyKDL.Vector(h1.x, h1.y, h1.z).Norm(), PyKDL.Vector(h2.x, h2.y, h2.z).Norm()) )
                    break

        return contacts

    # there is a lot of bugs in this function!
    def getContactPoints(self, threshold, f1=True, f2=True, f3=True, palm=True):
        self.tactile_lock.acquire()
        index = copy.copy(self.tactile_data_index)
        max_value = copy.copy(self.max_tactile_value)
        self.tactile_lock.release()

        contacts = []
        forces = []
        if f1:
            for i in range(0, self.tactile_data_len-2):
                time = self.tactile_data[index][0]
                if self.listener.canTransform('torso_base', '/'+self.prefix+'_HandFingerOneKnuckleThreeLink', time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', '/'+self.prefix+'_HandFingerOneKnuckleThreeLink', time))
                    for i in range(0, len(self.pressure_frames)):
                        if self.tactile_data[index][1][i] > threshold:
                            contacts.append( T_B_F * self.pressure_frames[i] * PyKDL.Vector() )
                            forces.append(self.tactile_data[index][4][i])
                    break
                index -= 1
                if index < 0:
                    index = copy.copy(self.tactile_data_len)-1

        if f2:
            for i in range(0, self.tactile_data_len-2):
                time = self.tactile_data[index][0]
                if self.listener.canTransform('torso_base', '/'+self.prefix+'_HandFingerTwoKnuckleThreeLink', time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', '/'+self.prefix+'_HandFingerTwoKnuckleThreeLink', time))
                    for i in range(0, len(self.pressure_frames)):
                        if self.tactile_data[index][2][i] > threshold:
                            contacts.append( T_B_F * self.pressure_frames[i] * PyKDL.Vector() )
                            forces.append(self.tactile_data[index][4][i])
                    break
                index -= 1
                if index < 0:
                    index = copy.copy(self.tactile_data_len)-1

        if f3:
            for i in range(0, self.tactile_data_len-2):
                time = self.tactile_data[index][0]
                if self.listener.canTransform('torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', time))
                    for i in range(0, len(self.pressure_frames)):
                        if self.tactile_data[index][3][i] > threshold:
                            contacts.append( T_B_F * self.pressure_frames[i] * PyKDL.Vector() )
                            forces.append(self.tactile_data[index][4][i])
                    break
                index -= 1
                if index < 0:
                    index = copy.copy(self.tactile_data_len)-1

        if palm:
            for i in range(0, self.tactile_data_len-2):
                time = self.tactile_data[index][0]
                if self.listener.canTransform('torso_base', '/'+self.prefix+'_HandPalmLink', time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', '/'+self.prefix+'_HandPalmLink', time))
                    for i in range(0, len(self.palm_pressure_frames)):
                        if self.tactile_data[index][4][i] > threshold:
                            contacts.append( T_B_F * self.palm_pressure_frames[i] * PyKDL.Vector() )
                            forces.append(self.tactile_data[index][4][i])
                    break
                index -= 1
                if index < 0:
                    index = copy.copy(self.tactile_data_len)-1

        return contacts, forces

