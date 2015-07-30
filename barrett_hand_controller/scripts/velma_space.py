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

import roslib
roslib.load_manifest('barrett_hand_controller')

import rospy
import tf

import ar_track_alvar_msgs.msg
from ar_track_alvar_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from barrett_hand_controller_msgs.msg import *
from barrett_hand_controller_msgs.srv import *
from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from threading import Lock

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
from numpy import *
import numpy as np
import copy
import matplotlib.pyplot as plt
import thread
from velma import Velma
from velmasim import VelmaSim
import random
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import velmautils
import openraveinstance
import itertools
import dijkstra
import grip
import operator

# reference frames:
# B - robot's base
# R - camera
# W - wrist
# E - gripper
# F - finger distal link
# T - tool
# C - current contact point
# N - the end point of finger's nail
# J - jar marker frame (jar cap)

class GraspingTask:
    """
Class for grasp learning.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = pub_marker
        self.listener = tf.TransformListener();

    def getMarkerPose(self, marker_id, wait = True, timeBack = None):
        try:
            marker_name = 'ar_marker_'+str(int(marker_id))
            if wait:
                self.listener.waitForTransform('torso_base', marker_name, rospy.Time.now(), rospy.Duration(4.0))
            if timeBack != None:
                time = rospy.Time.now() - rospy.Duration(timeBack)
            else:
                time = rospy.Time(0)
            jar_marker = self.listener.lookupTransform('torso_base', marker_name, time)
        except:
            return None
        return pm.fromTf(jar_marker)

    def getCameraPose(self):
        try:
            self.listener.waitForTransform('torso_base', "camera", rospy.Time.now(), rospy.Duration(4.0))
            T_B_C_tf = self.listener.lookupTransform('torso_base', "camera", rospy.Time(0))
        except:
            return None
        return pm.fromTf(T_B_C_tf)

    def getMarkerPoseFake(self, marker_id, wait = True, timeBack = None):
        T_B_Tm = PyKDL.Frame( PyKDL.Vector(0.55,-0.4,0.9) )
        T_B_Tbb = PyKDL.Frame( PyKDL.Vector(0.5,-0.8,2.0) )
        T_Tm_Bm = PyKDL.Frame( PyKDL.Vector(-0.06, 0.3, 0.135) )
        T_Bm_Gm = PyKDL.Frame( PyKDL.Rotation.RotZ(-30.0/180.*math.pi), PyKDL.Vector(0.1,-0.1,0.06) )
        if marker_id == 6:
            return T_B_Tm
        elif marker_id == 7:
            return T_B_Tm * T_Tm_Bm
        elif marker_id == 8:
            return T_B_Tbb
        elif marker_id == 19:
            return T_B_Tm * T_Tm_Bm * T_Bm_Gm
        elif marker_id == 35:
            return T_B_Tm * T_Tm_Bm * T_Bm_Gm
        return None

    def poseUpdaterThread(self, args, *args2):
        index = 0
        z_limit = 0.3
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.allow_update_objects_pose == None or not self.allow_update_objects_pose:
                continue
            for obj in self.objects:
                visible_markers_Br_Co = []
                visible_markers_weights_ori = []
                visible_markers_weights_pos = []
                visible_markers_idx = []
                for marker in obj.markers:
                    T_Br_M = self.getMarkerPose(marker[0], wait = False, timeBack = 0.3)
                    if T_Br_M != None and self.velma != None:
                        T_B_C = self.velma.T_B_C #self.getCameraPose()
                        T_C_M = T_B_C.Inverse() * T_Br_M
                        v = T_C_M * PyKDL.Vector(0,0,1) - T_C_M * PyKDL.Vector()
                        if v.z() > -z_limit:
                            continue
                        # v.z() is in range (-1.0, -0.3)
                        weight = ((-v.z()) - z_limit)/(1.0-z_limit)
                        if weight > 1.0 or weight < 0.0:
                            print "error: weight==%s"%(weight)
                        T_Co_M = marker[1]
                        T_Br_Co = T_Br_M * T_Co_M.Inverse()
                        visible_markers_Br_Co.append(T_Br_Co)
                        visible_markers_weights_ori.append(1.0-weight)
                        visible_markers_weights_pos.append(weight)
                        visible_markers_idx.append(marker[0])
                if len(visible_markers_Br_Co) > 0:
#                    if obj.name == "object":
#                        print "vis: %s"%(visible_markers_idx)
#                        print "w_o: %s"%(visible_markers_weights_ori)
#                        print "w_p: %s"%(visible_markers_weights_pos)

                    # first calculate mean pose without weights
                    R_B_Co = velmautils.meanOrientation(visible_markers_Br_Co)[1]
                    p_B_Co = velmautils.meanPosition(visible_markers_Br_Co)
                    T_B_Co = PyKDL.Frame(copy.deepcopy(R_B_Co.M), copy.deepcopy(p_B_Co))
                    distances = []
                    for m_idx in range(0, len(visible_markers_Br_Co)):
                        diff = PyKDL.diff(T_B_Co, visible_markers_Br_Co[m_idx])
                        distances.append( [diff, m_idx] )
                    Br_Co_list = []
                    weights_ori = []
                    weights_pos = []
                    for d in distances:
                        if d[0].vel.Norm() > 0.04 or d[0].rot.Norm() > 15.0/180.0*math.pi:
                            continue
                        Br_Co_list.append( visible_markers_Br_Co[d[1]] )
                        weights_ori.append( visible_markers_weights_ori[d[1]] )
                        weights_pos.append( visible_markers_weights_pos[d[1]] )

                    if len(Br_Co_list) > 0:
                        R_B_Co = velmautils.meanOrientation(Br_Co_list, weights=weights_ori)[1]
                        p_B_Co = velmautils.meanPosition(Br_Co_list, weights=weights_pos)
                        obj.updatePose( PyKDL.Frame(copy.deepcopy(R_B_Co.M), copy.deepcopy(p_B_Co)) )
                        self.openrave.updatePose(obj.name, T_Br_Co)

            index += 1
            if index >= 100:
                index = 0

    def allowUpdateObjects(self):
        self.allow_update_objects_pose = True

    def disallowUpdateObjects(self):
        self.allow_update_objects_pose = False

    def waitForOpenraveInit(self):
        while not rospy.is_shutdown():
            if self.openrave.rolling:
                break
            rospy.sleep(0.5)

    def switchToJoint(self, robot):
        if robot.isCartesianImpedanceActive():
            raw_input("Press Enter to enable joint impedance...")
            if robot.checkStopCondition():
                exit(0)
            robot.switchToJoint()
        elif robot.isJointImpedanceActive():
            pass
        else:
            print "FATAL ERROR: impedance control in unknown state: %s %s"%(robot.joint_impedance_active, robot.cartesian_impedance_active)
            exit(0)

    def switchToCartesian(self, robot):
        if robot.isJointImpedanceActive():
            raw_input("Press Enter to enable cartesian impedance...")
            if robot.checkStopCondition():
                exit(0)
            robot.switchToCart()
        elif robot.isCartesianImpedanceActive():
            pass
        else:
            print "FATAL ERROR: impedance control in unknown state: %s %s"%(robot.joint_impedance_active, robot.cartesian_impedance_active)
            exit(0)

    def spin(self):

        # load and init ik solver for right hand
        velma_ikr = velmautils.VelmaIkSolver()
        velma_ikr.initIkSolver()

        # create Openrave interface
        self.openrave = openraveinstance.OpenraveInstance(PyKDL.Frame(PyKDL.Vector(0,0,0.1)))
        self.openrave.startNewThread()

        self.waitForOpenraveInit()

        print "openrave initialised"

        # create the robot interface for simulation
        velma = VelmaSim(self.openrave, velma_ikr)

        normals = velmautils.generateNormalsSphere(60.0/180.0*math.pi)
        frames = velmautils.generateFramesForNormals(60.0/180.0*math.pi, normals)

        distance = 0.1
        space = []
        max_solutions = 0
	for x in np.arange(0.0, 1.01, distance):
            for y in np.arange(-1.0, 1.01, distance):
                for z in np.arange(1.0, 2.01, distance):
                    solutions = 0
                    for fr in frames:
                        T_Br_E = PyKDL.Frame(PyKDL.Vector(x,y,z)) * fr
                        if self.openrave.findIkSolution(T_Br_E) != None:
                            solutions += 1
                    space.append([x,y,z,solutions])
                    if solutions > max_solutions:
                        max_solutions = solutions
            print "x: %s"%(x)

        print "points: %s"%(len(space))
        m_id = 0
        for s in space:
            value = float(s[3]) / float(max_solutions)
            m_id = publishSinglePointMarker(PyKDL.Vector(s[0], s[1], s[2]), m_id, r=1, g=value, b=value, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(distance*value, distance*value, distance*value), T=None)
            if m_id%100 == 0:
                rospy.sleep(0.1)

        print "done"

        return

if __name__ == '__main__':

    rospy.init_node('grasp_leanring')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


