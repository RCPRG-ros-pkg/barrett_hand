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

import ar_track_alvar.msg
from ar_track_alvar.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *
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
from scipy import optimize
from velma import Velma
import random
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import velmautils
import openraveinstance

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

class Grip:
    def __init__(self):
        pass

    def addContact(self, T_O_C):
        pass


class GraspingTask:
    """
Class for grasp learning.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = pub_marker
        self.listener = tf.TransformListener();

    def getMarkerPose(self, marker_id):
        try:
            marker_name = 'ar_marker_'+str(int(marker_id))
            self.listener.waitForTransform('torso_base', marker_name, rospy.Time.now(), rospy.Duration(4.0))
            jar_marker = self.listener.lookupTransform('torso_base', marker_name, rospy.Time(0))
        except:
            return None
        return pm.fromTf(jar_marker)

    def poseUpdaterThread(self, args, *args2):
        while not rospy.is_shutdown():
            for obj in self.objects:
                T_Br_M = self.getMarkerPose(obj[0])
                if T_Br_M != None:
                    T_Bo_M = obj[2]
                    T_Br_Bo = T_Br_M * T_Bo_M.Inverse()
                    self.openrave.updatePose(obj[1], T_Br_Bo)
            rospy.sleep(0.1)

    def spin(self):
        # create the robot interface
        velma = Velma()

        self.openrave = openraveinstance.OpenraveInstance(velma, PyKDL.Frame(PyKDL.Vector(0,0,0.1)))
        self.openrave.startNewThread()

        while not rospy.is_shutdown():
            if self.openrave.rolling:
                break
            rospy.sleep(0.5)

        self.objects = [
        [6, "table", PyKDL.Frame(PyKDL.Vector(0, -0.225, 0.035)), "box", [0.60,0.85,0.07] ],
        [7, "box", PyKDL.Frame(PyKDL.Vector(-0.07, 0.085, 0.065)), "box", [0.22,0.24,0.135] ],
        [5, "object", PyKDL.Frame(PyKDL.Vector(0.0, 0.071, 0.030)), "box", [0.060,0.354,0.060] ],
        ]

        for obj in self.objects:
            if obj[3] == "box":
                self.openrave.addBox(obj[1], obj[4][0], obj[4][1], obj[4][2])

        # test surface sampling
        if False:
            vertices, indices = self.openrave.getMesh("object")
            print vertices
            print indices
            points = velmautils.sampleMesh(vertices, indices, 0.002, [PyKDL.Vector(0.00,0,0.00)], 0.04)
            print len(points)
            m_id = 0
            m_id = self.pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
            raw_input("Press Enter to continue...")

            rospy.sleep(5.0)

            pt_list = []
            for i in range(0, 20):
                pt_list.append(PyKDL.Vector((1.0*i/20.0)*0.1-0.05, 0, 0))
            points = velmautils.sampleMesh(vertices, indices, 0.002, pt_list, 0.01)
            print len(points)
            m_id = 0
            m_id = self.pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
            rospy.sleep(1.0)
            fr = velmautils.estPlane(points)
            m_id = self.pub_marker.publishFrameMarker(fr, m_id)
            rospy.sleep(1.0)
            exit(0)

        if False:
            m_id = 0
            velma.updateTransformations()
            finger = 0
            center = PyKDL.Vector(0.05,-0.01,0)
            T_E_Fi3 = [velma.T_E_F13, velma.T_E_F23, velma.T_E_F33]
            T_Fi3_E = [velma.T_F13_E, velma.T_F23_E, velma.T_F33_E]
            centers = [velma.T_B_W * velma.T_W_E * velma.T_E_F13 * center, velma.T_B_W * velma.T_W_E * velma.T_E_F23 * center, velma.T_B_W * velma.T_W_E * velma.T_E_F33 * center]
            for c in centers:
                if c != None:
                    c_Fi3 = T_Fi3_E[finger] * velma.T_E_W * velma.T_B_W.Inverse() * c
                    pt_list = []
                    for angle in np.linspace(velma.q_rf[finger*3 + 1]-0.0/180.0*math.pi, velma.q_rf[finger*3 + 1]+10.0/180.0*math.pi, 20):
                        T_E_F = velma.get_T_E_Fd(finger, angle, 0)
                        cn_B = velma.T_B_W * velma.T_W_E * T_E_F * c_Fi3
                        pt_list.append(cn_B)
                    m_id = self.pub_marker.publishMultiPointsMarker(pt_list, m_id, r=1, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.004, 0.004, 0.004))#, T=T_Br_O)
                finger += 1
            exit(0)

        # start thread for updating objects' positions in openrave
        thread.start_new_thread(self.poseUpdaterThread, (None,1))

        velma.updateTransformations()

        k_jar_touching = Wrench(Vector3(1200.0, 1200.0, 1200.0), Vector3(300.0, 300.0, 300.0))

        if True:
            # reset the gripper
            velma.reset_fingers()
            velma.calibrate_tactile_sensors()
            velma.set_median_filter(8)

            # start with very low stiffness
            print "setting stiffness to very low value"
            velma.moveImpedance(velma.k_error, 0.5)
            if velma.checkStopCondition(0.5):
                exit(0)

            raw_input("Press Enter to continue...")
            if velma.checkStopCondition():
                exit(0)

            velma.updateTransformations()
            velma.updateAndMoveTool( velma.T_W_E, 1.0 )
            if velma.checkStopCondition(1.0):
                exit(0)

            raw_input("Press Enter to continue...")
            print "setting stiffness to bigger value"

            velma.moveImpedance(k_jar_touching, 3.0)
            if velma.checkStopCondition(3.0):
                exit(0)

        grasps,indices = self.openrave.generateGrasps("object")

        min_cost = 10000.0
        min_i = 0
        for i in range(0, len(grasps)):
            T_Br_E = self.openrave.getGraspTransform(grasps[i], collisionfree=True)
            velma.updateTransformations()
            traj_T_B_Ed = [T_Br_E]
            cost = velma.getTrajCost(traj_T_B_Ed, False, False)
            print "%s   cost: %s"%(i,cost)
            if cost < min_cost:
                min_cost = cost
                min_i = i

        if min_cost > 1000.0:
            print "could not reach the destination point"
            exit(0)

        T_Br_E = self.openrave.getGraspTransform(grasps[min_i], collisionfree=True)
        self.openrave.showGrasp(grasps[min_i])

        T_B_Wd = T_Br_E * velma.T_E_W
        duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
        velma.moveWrist2(T_B_Wd*velma.T_W_T)
        self.openrave.showTrajectory(T_Br_E, 3.0)

        print "standoff: %s"%(self.openrave.getGraspStandoff(grasps[min_i]))

        raw_input("Press Enter to move the robot in " + str(duration) + " s...")
        if velma.checkStopCondition():
            exit(0)
        velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
        if velma.checkStopCondition(duration):
            exit(0)

        raw_input("Press Enter to close fingers...")

        # close the fingers for grasp
        velma.move_hand_client((120.0/180.0*math.pi,120.0/180.0*math.pi,120.0/180.0*math.pi,0), v=(0.5, 0.5, 0.5, 1.0), t=(2000.0, 2000.0, 2000.0, 2000.0))
        m_id = 0
        if True:
            if velma.checkStopCondition(6.0):
                exit(0)
        else:
            time_end = rospy.Time.now() + rospy.Duration(6.0)
            all_contacts = []
            all_forces = []
            while rospy.Time.now() < time_end:
                contacts = [[],[],[]]
                forces = [[],[],[]]
                contacts[0], forces[0] = velma.getContactPoints(100, f1=True, f2=False, f3=False, palm=False)
                contacts[1], forces[1] = velma.getContactPoints(100, f1=False, f2=True, f3=False, palm=False)
                contacts[2], forces[2] = velma.getContactPoints(100, f1=False, f2=False, f3=True, palm=False)
                if len(contacts) > 0:
                    all_contacts.append(contacts)
                    all_forces.append(forces)
                rospy.sleep(0.01)
                if velma.checkStopCondition():
                    exit(0)
            for c in all_contacts:
                m_id = self.pub_marker.publishMultiPointsMarker(c[0], m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
                rospy.sleep(0.01)
                m_id = self.pub_marker.publishMultiPointsMarker(c[1], m_id, r=0, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
                rospy.sleep(0.01)
                m_id = self.pub_marker.publishMultiPointsMarker(c[2], m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
                rospy.sleep(0.01)

        # get contact points and forces for each finger
        velma.updateTransformations()
        contacts = [[],[],[]]
        forces = [[],[],[]]
        contacts[0], forces[0] = velma.getContactPoints(100, f1=True, f2=False, f3=False, palm=False)
        contacts[1], forces[1] = velma.getContactPoints(100, f1=False, f2=True, f3=False, palm=False)
        contacts[2], forces[2] = velma.getContactPoints(100, f1=False, f2=False, f3=True, palm=False)

        m_id = self.pub_marker.publishMultiPointsMarker(contacts[0], m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
        rospy.sleep(0.01)
        m_id = self.pub_marker.publishMultiPointsMarker(contacts[1], m_id, r=0, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
        rospy.sleep(0.01)
        m_id = self.pub_marker.publishMultiPointsMarker(contacts[2], m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
        rospy.sleep(0.01)

        # calculate force-weighted center of contacts for each finger
        centers = []
        forces_sum = []
        for finger in range(0, len(contacts)):
            center = PyKDL.Vector()
            force_sum = 0.0
            for i in range(0, len(contacts[finger])):
                center += contacts[finger][i] * forces[finger][i]
                force_sum += forces[finger][i]
            forces_sum.append(force_sum)
            if force_sum > 0.0:
                center *= (1.0/force_sum)
                centers.append(center)
            else:
                centers.append(None)

        print "fingers in contact: %s"%(len(centers))

        for c in centers:
            if c != None:
                m_id = self.pub_marker.publishSinglePointMarker(c, m_id, r=1, g=1, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))

        T_Br_O = self.openrave.getPose("object")
        T_O_Br = T_Br_O.Inverse()
        vertices, indices = self.openrave.getMesh("object")
        finger = 0
        T_E_Fi3 = [velma.T_E_F13, velma.T_E_F23, velma.T_E_F33]
        T_Fi3_E = [velma.T_F13_E, velma.T_F23_E, velma.T_F33_E]
        actual_angles = [velma.q_rf[1], velma.q_rf[4], velma.q_rf[6]]
        for c in centers:
            if c != None:
                c_Fi3 = T_Fi3_E[finger] * velma.T_E_W * velma.T_B_W.Inverse() * c
                pt_list = []
                for angle in np.linspace(actual_angles[finger]-10.0/180.0*math.pi, actual_angles[finger]+10.0/180.0*math.pi, 20):
                    T_E_F = velma.get_T_E_Fd(finger, angle, 0)
                    cn_B = velma.T_B_W * velma.T_W_E * T_E_F * c_Fi3
                    cn_O = T_O_Br * cn_B
                    pt_list.append(cn_O)
                m_id = self.pub_marker.publishMultiPointsMarker(pt_list, m_id, r=1, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.004, 0.004, 0.004), T=T_Br_O)
                points = velmautils.sampleMesh(vertices, indices, 0.002, pt_list, 0.01)
                print len(points)
                m_id = self.pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.004, 0.004, 0.004), T=T_Br_O)
                rospy.sleep(1.0)
                fr = velmautils.estPlane(points)
                m_id = self.pub_marker.publishFrameMarker(T_Br_O*fr, m_id)
                rospy.sleep(1.0)

            finger += 1

        raw_input("Press Enter to open fingers...")

        velma.move_hand_client((0,0,0,0), v=(1.2, 1.2, 1.2, 1.2), t=(2000.0, 2000.0, 2000.0, 2000.0))
        if velma.checkStopCondition(3.0):
            exit(0)

        print "setting stiffness to very low value"
        velma.moveImpedance(velma.k_error, 0.5)
        if velma.checkStopCondition(0.5):
            exit(0)

        while not rospy.is_shutdown():
            rospy.sleep(1.0)

        exit(0)

if __name__ == '__main__':

    rospy.init_node('grasp_leanring')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


