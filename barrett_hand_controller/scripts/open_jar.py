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

class MarkerPublisher:
    def __init__(self):
        self.pub_marker = rospy.Publisher('/velma_markers', MarkerArray)

    def publishSinglePointMarker(self, pt, i, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005)):
        m = MarkerArray()
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = i
        marker.type = m_type
        marker.action = 0
        marker.pose = Pose( Point(pt.x(),pt.y(),pt.z()), Quaternion(0,0,0,1) )
        marker.scale = scale
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)
        self.pub_marker.publish(m)

    def publishMultiPointsMarker(self, pt, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002)):
        m = MarkerArray()
        for i in range(0, len(pt)):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = i
            marker.type = m_type
            marker.action = 0
            marker.pose = Pose( Point(pt[i].x(),pt[i].y(),pt[i].z()), Quaternion(0,0,0,1) )
            marker.scale = scale
            marker.color = ColorRGBA(r,g,b,0.5)
            m.markers.append(marker)
        self.pub_marker.publish(m)

    def publishVectorMarker(self, v1, v2, i, r, g, b, frame='torso_base', namespace='default', scale=0.001):
        m = MarkerArray()
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = 0
        marker.points.append(Point(v1.x(), v1.y(), v1.z()))
        marker.points.append(Point(v2.x(), v2.y(), v2.z()))
        marker.pose = Pose( Point(0,0,0), Quaternion(0,0,0,1) )
        marker.scale = Vector3(scale, 2.0*scale, 0)
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)
        self.pub_marker.publish(m)

    def publishFrameMarker(self, T, base_id, scale=0.1, frame='torso_base', namespace='default'):
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(scale,0,0), base_id, 1, 0, 0, frame, namespace)
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(0,scale,0), base_id+1, 0, 1, 0, frame, namespace)
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(0,0,scale), base_id+2, 0, 0, 1, frame, namespace)
        return base_id+3

def getAngle(v1, v2):
    return math.atan2((v1*v2).Norm(), PyKDL.dot(v1,v2))

class Jar:

    def generatePoints(self, distance):
        self.pt = []
        # side surface
        L = 2.0 * math.pi * self.R
        L_count = int( L/distance )
        H_count = int( self.H/distance )
        for l in range(0, L_count):
            for h in range(1, H_count):
               angle = 2.0*math.pi*float(l)/L_count
               self.pt.append(PyKDL.Vector(self.R*math.cos(angle), self.R*math.sin(angle), self.H*float(h)/H_count)) 
        # top and bottom surface
#        R_count = int( self.R/distance )
#        for r in range(0, R_count+1):
#            current_r = (float(r)/R_count)*self.R
#            L = 2.0*math.pi*current_r
#            L_count = int( L/distance ) + 1
#            for l in range(0, L_count):
#               angle = 2.0*math.pi*float(l)/L_count
#               self.pt.append(PyKDL.Vector(current_r*math.cos(angle), current_r*math.sin(angle), 0.0)) 
#               self.pt.append(PyKDL.Vector(current_r*math.cos(angle), current_r*math.sin(angle), self.H)) 

    def __init__(self, pub_marker = None):
        self.pub_marker = pub_marker
        self.R = 0.04
        self.H = 0.195
        self.generatePoints(0.01)
        self.T_B_Jbase = PyKDL.Frame(PyKDL.Vector(0,0,0))
        self.T_Jbase_Jmarker = PyKDL.Frame(PyKDL.Vector(0,0,self.H))
        self.T_Jmarker_Jbase = self.T_Jbase_Jmarker.Inverse()
        self.position_error = 5.0   # in meters: 5.0m -> "there is a jar somewhere in this room"
        self.resetContactObservations()

    def drawPoints(self):
        print "drawPoints: %s"%(len(self.pt))
        if self.pub_marker != None:
            self.pub_marker.publishMultiPointsMarker(self.pt, 0, 1, 0, namespace="jar_points", frame_id="jar")

    def publishTf(self):
        pose = pm.toMsg(self.T_B_Jbase)
        br.sendTransform([pose.position.x, pose.position.y, pose.position.z], [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], rospy.Time.now(), "jar", "torso_base")

    def tfBroadcasterLoop(self, interval, *args):
        try:
            while not rospy.is_shutdown():
                self.publishTf()
                self.drawJar()
                rospy.sleep(interval)
        except:
            print "Catched tfBroadcasterLoop exception. Stopped drawing the jar model."

    def drawJar(self):
        if self.pub_marker != None:
            self.pub_marker.publishSinglePointMarker(PyKDL.Vector(0,0,self.H*0.5), 0, r=0, g=1, b=0, namespace='jar', frame_id='jar', m_type=Marker.CYLINDER, scale=Vector3(self.R*2.0, self.R*2.0, self.H))

    def addMarkerObservation(self, T_B_M):
        self.position_error = 0.05   # in meters: 0.05m -> "there is a jar somewhere around this marker"
        self.T_B_Jbase = T_B_M * self.T_Jmarker_Jbase

    def resetContactObservations(self):
        self.contacts_Jbase = []

    def addContactObservation(self, pt_B):
        pt_Jbase = self.T_B_Jbase.Inverse() * pt_B
        self.contacts_Jbase.append(copy.copy(pt_Jbase))

    def drawContactObservations(self):
        print "drawContactObservations: %s"%(len(self.contacts_Jbase))
        if self.pub_marker != None:
            self.pub_marker.publishMultiPointsMarker(self.contacts_Jbase, 1, 0, 0, namespace="jar_obs", frame_id="jar")

    def estPosition(self):
        def calc_R(xo, yo):
            ret = []
            """ calculate the minimum distance of each contact point from jar surface pt """
            o = PyKDL.Vector(xo, yo, 0)
            for contact in self.contacts_Jbase:
                contact_o = contact + o
                min_dist = 1000000.0
                for p in self.pt:
                    dist = (contact_o-p).Norm()
                    if dist < min_dist:
                        min_dist = dist
                #ret.append( math.sqrt(min_dist) )
                ret.append( min_dist )
            return numpy.array(ret)
        
        def f_2(c):
            """ calculate the algebraic distance between each contact point and jar surface pt """
            Di = calc_R(*c)
            return Di

        position_estimate = 0.0, 0.0#, 0.0
        position_2, ier = optimize.leastsq(f_2, position_estimate, maxfev = 1000)

        return PyKDL.Vector(position_2[0], position_2[1],0)

    def processContactObservations(self):
        position = self.estPosition()
        print position
        for i in range(0, len(self.contacts_Jbase)):
            self.contacts_Jbase[i] += position
        self.T_B_Jbase = copy.deepcopy( self.T_B_Jbase * PyKDL.Frame(-position) )

    def processContactObservationsForTop(self, top_offset=0.0):
        if len(self.contacts_Jbase) < 1:
            return
        max_z = 0.0
        for i in range(0, len(self.contacts_Jbase)):
            if self.contacts_Jbase[i].z() > max_z:
                max_z = self.contacts_Jbase[i].z() + top_offset
        position = PyKDL.Vector(0,0,self.H-max_z)
        for i in range(0, len(self.contacts_Jbase)):
            self.contacts_Jbase[i] += position
        self.T_B_Jbase = copy.deepcopy( self.T_B_Jbase * PyKDL.Frame(-position) )

    def getJarCapFrame(self):
        return self.T_B_Jbase * self.T_Jbase_Jmarker

class Grasp:
    def __init__(self, T_E_Ge, name, q, q_pre, t=(3000, 3000, 3000, 3000), t_pre=(3000, 3000, 3000, 3000)):
        # E is the gripper frame, G is the object's grasp frame
        self.T_E_Ge = T_E_Ge
        if self.T_E_Ge == None:
            self.T_Ge_E = None
        else:
            self.T_Ge_E = self.T_E_Ge.Inverse()
        self.name = name
        self.q_pre = q_pre
        self.q = q
        self.t = t
        self.t_pre = t_pre

class Relation:
    # stop_on_contact=[f1, f2, f3, palm] (bool)
    def __init__(self, name, grasp, T_Go_Ge, on_success, on_failure, stop_on_contact=[False, False, False, False], collect_contacts=False, max_v_l=0.1, max_v_r=0.2, k_pre=None, k_post=None):
        self.name = name
        self.grasp = grasp
        self.T_Go_Ge = T_Go_Ge
        self.on_success = on_success
        self.on_failure = on_failure
        self.stop_on_contact = stop_on_contact
        self.collect_contacts = collect_contacts
        self.max_v_l = max_v_l
        self.max_v_r = max_v_r
        self.k_pre = k_pre
        self.k_post = k_post

class Action:
    def __init__(self, velma):
        self.velma = velma
        self.relations = []

    def getNumberOfRelations(self):
        return len(self.relations)

    def setTarget(self, T_B_JC):
        pass

    def getRelIndex(self, name):
        i = 0
        for r in self.relations:
            if r.name == name:
                return i
            i += 1
        return -1

class RemoveJarLidAction(Action):
    def __init__(self, velma):
        Action.__init__(self, velma)
        self.relations = [None, None, None]

    def setTarget(self, T_B_JC):
        # recalculate grasp pose
        self.T_B_Go = T_B_JC * PyKDL.Frame(PyKDL.Rotation.RotY(-180.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Vector(0,0,0.01))

    def getBestGraspForCurrentPose(self, grasps):
        # find the best angle for rotating the cap
        # iterate through angles
        best_score = 1000000.0
        self.velma.updateTransformations()
        best_grasp = None
        for angle_cap in [180.0/180.0*math.pi, 120.0/180.0*math.pi, 90.0/180.0*math.pi, 60.0/180.0*math.pi, 45.0/180.0*math.pi, 30.0/180.0*math.pi]:
            count = int(angle_cap/(5.0/180.0*math.pi))
            if count < 2:
                count = 2
            angles = np.linspace(angle_cap, 0.0, count)
            for grasp in grasps:
                if grasp.name == "JarCapForce":
                    # simulate the approach
                    traj_T_B_Ed = []
                    # calculate the transform
                    T_B_Ed = self.T_B_Go * PyKDL.Frame() * grasp.T_Ge_E
                    traj_T_B_Ed.append(T_B_Ed)
                    T_B_Ed_last = traj_T_B_Ed[-1]
                    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    cost = self.velma.getTrajCost(traj_T_B_Ed, True, False, q_end=q)
                    print "cost1: %s"%(cost)
                    if cost > 1000.0:
                        continue
                    # simulate the approach
                    traj_T_B_Ed = []
                    for beta in angles:
                        # calculate the transform
                        T_B_Ed = self.T_B_Go * PyKDL.Frame(PyKDL.Rotation.RotZ(beta)) * grasp.T_Ge_E
                        traj_T_B_Ed.append(T_B_Ed)
                    cost += self.velma.getTrajCost(traj_T_B_Ed, False, False, q_start=q, T_B_Eprev=T_B_Ed_last)
                    print "cost2: %s"%(cost)
                    if cost > 1000.0:
                        continue
                    # prefer the pose with the smallest twist to the current pose
                    if cost < best_score:
                        best_score = cost
                        best_grasp = grasp
            if best_score > 1000.0:
                print "it is impossible to reach the jar for rotation angle %s deg"%(angle_cap/math.pi*180.0)
            else:
                break
        if best_score > 1000.0:
            return [None,[]]
        return [best_grasp, angles]

    def recalculateRelations(self, grasps):
        [best_grasp, angles] = self.getBestGraspForCurrentPose(grasps)
        grasp0 = copy.deepcopy(best_grasp)
        grasp0.q_pre = [0,0,0,0]
        grasp0.t_pre=(3000, 3000, 3000, 3000)
        grasp0.q = None
        grasp1 = copy.deepcopy(best_grasp)
        grasp1.t=(3000, 3000, 3000, 3000)
        grasp1.t_pre=(3000, 3000, 3000, 3000)
        grasp2 = copy.deepcopy(best_grasp)
        grasp2.q_pre = [grasp1.q[0]+10.0/180.0*math.pi, grasp1.q[1]+10.0/180.0*math.pi, grasp1.q[2]+10.0/180.0*math.pi, grasp1.q[3]]
        grasp2.t_pre=(1000, 1000, 1000, 1000)
        grasp2.q = None
        pos_z = 0.0
        self.relations = []
        self.relations.append(
        Relation("reset_gripper", grasp0, PyKDL.Frame(PyKDL.Vector(0,0,-pos_z)) * PyKDL.Frame(PyKDL.Rotation.RotZ(angles[0])), "move_above", "RECALCULATE,reset_gripper", k_pre=Wrench(Vector3(1200.0, 1200.0, 1200.0), Vector3(300.0, 300.0, 300.0)) ) )
        self.relations.append(
        Relation("move_above", grasp1, PyKDL.Frame(PyKDL.Vector(0,0,-pos_z)) * PyKDL.Frame(PyKDL.Rotation.RotZ(angles[0])), "rotate_lid_0", "RECALCULATE,reset_gripper", k_pre=Wrench(Vector3(1200.0, 1200.0, 1200.0), Vector3(300.0, 300.0, 300.0)), k_post=Wrench(Vector3(500.0, 500.0, 1200.0), Vector3(300.0, 300.0, 300.0)) ) )
        i = 0
        for beta in angles:
            if beta == angles[-1]:
                next_state = "pull_lid"
            else:
                next_state = "rotate_lid_"+str(i+1)
            self.relations.append( Relation("rotate_lid_"+str(i), grasp2, PyKDL.Frame(PyKDL.Vector(0,0,-pos_z)) * PyKDL.Frame(PyKDL.Rotation.RotZ(beta)), next_state, "FAILURE", k_pre=Wrench(Vector3(100.0, 100.0, 1200.0), Vector3(300.0, 300.0, 300.0))) )
            i += 1

        self.relations.append( Relation("pull_lid", grasp2, PyKDL.Frame(PyKDL.Vector(0,0,-pos_z-0.05)) * PyKDL.Frame(PyKDL.Rotation.RotZ(beta)), "SUCCESS", "FAILURE", max_v_l=0.01, max_v_r=0.05, collect_contacts=True) )
        i += 1

class JarLidPinchGraspAction(Action):
    def __init__(self, velma):
        Action.__init__(self, velma)
        self.relations = [None, None, None]

    def setTarget(self, T_B_JC):
        # recalculate grasp pose
        self.T_B_Go = T_B_JC * PyKDL.Frame(PyKDL.Rotation.RotY(-180.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Vector(0,0,0.01))

    def getBestGraspForCurrentPose(self, grasps):
        # then, we search for the best angle about S.Z axis for jar cap approach
        # iterate through angles
        best_score = 1000000.0
        self.velma.updateTransformations()
        best_grasp = None
        for beta_diff in [90.0/180.0*math.pi, -90.0/180.0*math.pi]:#, 70.0/180.0*math.pi]:
            for grasp in grasps:
                if grasp.name == "JarCapPinch":
                    traj_T_B_Ed = []
                    # calculate the transform
                    T_B_Ed = self.T_B_Go * PyKDL.Frame() * grasp.T_Ge_E
                    traj_T_B_Ed.append(T_B_Ed)
                    T_B_Ed_last = traj_T_B_Ed[-1]
                    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    cost = self.velma.getTrajCost(traj_T_B_Ed, True, False, q_end=q)
                    print "cost1: %s"%(cost)
                    if cost > 1000.0:
                        continue

                    traj_T_B_Ed = []
                    T_B_Ed = self.T_B_Go * PyKDL.Frame(PyKDL.Rotation.RotZ(beta_diff)) * grasp.T_Ge_E
                    traj_T_B_Ed.append(T_B_Ed)
                    cost += self.velma.getTrajCost(traj_T_B_Ed, True, False, q_start=q, T_B_Eprev=T_B_Ed_last)
                    print "cost2: %s"%(cost)
                    if cost > 1000.0:
                        continue

                    # prefer the pose with the smallest twist to the current pose
                    if cost < best_score:
                        best_score = cost
                        best_grasp = grasp
            if best_score > 1000.0:
                print "it is impossible to reach the jar for angle %s deg"%(beta_diff/math.pi*180.0)
            else:
                break
        if best_score > 1000.0:
            return [None, 0]
        return [best_grasp, beta_diff]

    def recalculateRelations(self, grasps):
        [best_grasp, beta_diff] = self.getBestGraspForCurrentPose(grasps)
        grasp1 = copy.deepcopy(best_grasp)
        grasp1.t_pre=(3000, 3000, 3000, 3000)
        grasp1.q = None
        grasp2 = copy.deepcopy(best_grasp)
        grasp2.t_pre=(3000, 3000, 3000, 3000)
        grasp2.t=(2000, 2000, 2000, 2000)
        self.relations = []
        self.relations.append( Relation("move_above", grasp1, PyKDL.Frame(), "RECALCULATE,first_grip", "RECALCULATE,move_above", k_pre=Wrench(Vector3(1200.0, 1200.0, 1200.0), Vector3(300.0, 300.0, 300.0)) ) )
        self.relations.append( Relation("first_grip", copy.deepcopy(best_grasp), PyKDL.Frame(), "second_grip", "RECALCULATE,move_above", collect_contacts=True, k_pre=Wrench(Vector3(200.0, 200.0, 1200.0), Vector3(300.0, 300.0, 300.0)) ) )
        self.relations.append( Relation("second_grip", copy.deepcopy(best_grasp), PyKDL.Frame(PyKDL.Rotation.RotZ(beta_diff)), "SUCCESS", "second_grip2", collect_contacts=True) )
        self.relations.append( Relation("second_grip2", copy.deepcopy(best_grasp), PyKDL.Frame(PyKDL.Rotation.RotZ(beta_diff)), "SUCCESS", "FAILURE", collect_contacts=True) )

class JarLidTopTouchAction(Action):
    def __init__(self, velma):
        Action.__init__(self, velma)
        self.relations = [None, None, None]

    def setTarget(self, T_B_JC):
        # recalculate grasp pose
        self.T_B_Go = T_B_JC * PyKDL.Frame(PyKDL.Rotation.RotY(-180.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Vector(0,0,0.01))

    def getBestGraspForCurrentPose(self, grasps):
        # then, we search for the best angle about S.Z axis for jar cap approach
        # iterate through angles
        best_score = 1000000.0
        self.velma.updateTransformations()
        best_grasp = None
        for grasp in grasps:
            if grasp.name == "JarCapTopTouch":
                traj_T_B_Ed = []
                T_B_Ed = self.T_B_Go * self.relations[0].T_Go_Ge * grasp.T_Ge_E
                traj_T_B_Ed.append(T_B_Ed)
                T_B_Ed = self.T_B_Go * self.relations[1].T_Go_Ge * grasp.T_Ge_E
                traj_T_B_Ed.append(T_B_Ed)
                cost = self.velma.getTrajCost(traj_T_B_Ed, True, False)
                if cost < best_score:
                    best_score = cost
                    best_grasp = grasp
        if best_score > 1000.0:
            print "it is impossible to reach the jar"
            return None
        return best_grasp

    def recalculateRelations(self, grasps):
        self.relations[0] = Relation("move_above", None, PyKDL.Frame(PyKDL.Vector(0,0,-0.05)), "touch", "RECALCULATE,move_above", k_pre=Wrench(Vector3(1200.0, 1200.0, 1200.0), Vector3(300.0, 300.0, 300.0)))
        self.relations[1] = Relation("touch", None, PyKDL.Frame(PyKDL.Vector(0,0,0.05)), "return", "FAILURE", stop_on_contact=[True, True, True, False], collect_contacts=True, max_v_l=0.01, max_v_r=0.05)
        self.relations[2] = Relation("return", None, PyKDL.Frame(PyKDL.Vector(0,0,-0.05)), "SUCCESS", "FAILURE")
        best_grasp = self.getBestGraspForCurrentPose(grasps)
        self.relations[0].grasp = copy.deepcopy(best_grasp)
        self.relations[1].grasp = copy.deepcopy(best_grasp)
        self.relations[1].grasp.q_pre = None
        self.relations[1].grasp.q = None
        self.relations[2].grasp = copy.deepcopy(best_grasp)
        self.relations[2].grasp.q_pre = None
        self.relations[2].grasp.q = None

class JarOpener:
    """
Class for opening the jar.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = pub_marker
        self.listener = tf.TransformListener();
        self.joint_states_lock = Lock()

    def getJarMarkerPose(self):
        try:
            self.listener.waitForTransform('torso_base', 'ar_marker_0', rospy.Time.now(), rospy.Duration(4.0))
            jar_marker = self.listener.lookupTransform('torso_base', 'ar_marker_0', rospy.Time(0))
        except:
            return None
        return pm.fromTf(jar_marker)

    def resetGripper(self, robot):
        robot.move_hand_client("right", (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi) )
        if robot.checkStopCondition(3.0):
            exit(0)

    def testHandKinematics(self, robot):
        robot.move_hand_client("right", (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 40.0/180.0*numpy.pi) )
        if robot.checkStopCondition(3.0):
            exit(0)
        robot.updateTransformations()

        angle = -10.0
        while angle < 180.0:
            T_E_Fd = robot.get_T_E_Fd(0, math.pi*angle/180.0)
            pt = robot.T_B_W * robot.T_W_E * T_E_Fd * PyKDL.Vector(0.05, -0.01, 0)
            if self.pub_marker != None:
                self.pub_marker.publishSinglePointMarker(pt, 0, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))

            T_E_Fd = robot.get_T_E_Fd(1, math.pi*angle/180.0)
            pt = robot.T_B_W * robot.T_W_E * T_E_Fd * PyKDL.Vector(0.05, -0.01, 0)
            if self.pub_marker != None:
                self.pub_marker.publishSinglePointMarker(pt, 1, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))

            T_E_Fd = robot.get_T_E_Fd(2, math.pi*angle/180.0)
            pt = robot.T_B_W * robot.T_W_E * T_E_Fd * PyKDL.Vector(0.05, -0.01, 0)
            if self.pub_marker != None:
                self.pub_marker.publishSinglePointMarker(pt, 2, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))
            if robot.checkStopCondition(0.02):
                exit(0)
            angle += 0.5                

    def collectGripperKinematics(self, robot):
        if False:
            # reset the gripper
            self.resetGripper(robot)
            # get finger tip trajectory during finger movement
            robot.getFingersKinematics()
            # reset the gripper
            self.resetGripper(robot)
            tabs = [robot.F1_kinematics, robot.F2_kinematics, robot.F3_kinematics]
            for t in tabs:
                print "self.FX_kinematics=["
                for data in t:
                    q = data[1].M.GetQuaternion()
                    p = data[1].p
                    print "[%s,PyKDL.Frame(PyKDL.Rotation.Quaternion(%s,%s,%s,%s),PyKDL.Vector(%s,%s,%s))],"%(data[0], q[0], q[1], q[2], q[3], p.x(), p.y(), p.z() )
                print "]"
            exit(0)

    def resetContacts(self):
         self.contacts = []

    def addContact(self, P_contact):
        if len(self.contacts) > 0:
            min_dist = 1000000.0
            for c in self.contacts:
                dist = (c-P_contact).Norm()
                if dist < min_dist:
                    min_dist = dist
            if min_dist > 0.002:
                self.contacts.append(P_contact)
        else:
            self.contacts.append(P_contact)

    def jointStatesCallback(self, data):
        if len(data.name) == 16:
            if data.name[7] == 'right_arm_5_joint':
                self.joint_states_lock.acquire()
                self.q5 = data.position[7]
                self.joint_states_lock.release()

    def generateGrasps_jarCapForce(self, velma, jar, spread_angle):
        velma.updateTransformations()

        # we move the jar along y axis i E frame
        # search for such configuration, that every finger touches the jar in the central plane of the finger
        best_n = 1000.0
        x = 0.0
        for y in np.arange(-0.04, 0.04, 0.001):
            kinematics = [velma.F1_kinematics, velma.F2_kinematics, velma.F3_kinematics]
            finger_index = 0
            contact = [False, False, False]
            desired_angles = [0.0, 0.0, 0.0]
            pt = [PyKDL.Vector(), PyKDL.Vector(), PyKDL.Vector()]
            T_E_Fd = [PyKDL.Frame(), PyKDL.Frame(), PyKDL.Frame()]
            # for each finger iterate through joint angles until there is a contact with the jar
            for k in kinematics:
                # calculate contact points we want to reach
                for data in k:
                    desired_angles[finger_index] = data[0]
                    T_E_Fd[finger_index] = velma.get_T_E_Fd(finger_index, data[0], spread_angle)
                    # iterate through all cells of the sensor matrix
                    for T_F_S in velma.pressure_frames:
                        pt[finger_index] = T_E_Fd[finger_index] * T_F_S * PyKDL.Vector()
                        z = pt[finger_index].z()
                        r = (pt[finger_index]-PyKDL.Vector(x,y,z)).Norm()
                        if r <= jar.R:
                            contact[finger_index] = True
                            break
                    if contact[finger_index]:
                        break
                # if we do not have contact for one finger, discard the whole try
                if not contact[finger_index]:
                    break
                finger_index += 1
            # if we have contact point for each finger, compute the plane for three points and its normal
            if contact[0] and contact[1] and contact[2]:
                center_E = PyKDL.Vector(x,y,0)
                center_F = T_E_Fd[0].Inverse() * center_E
                n = math.fabs(center_F.z())
                center_F = T_E_Fd[1].Inverse() * center_E
                n += math.fabs(center_F.z())
                center_F = T_E_Fd[2].Inverse() * center_E
                n += math.fabs(center_F.z())
                if n < best_n:
                    best_n = n
                    best_y = y
                    best_pt = copy.deepcopy(pt)
                    best_desired_angles = copy.deepcopy(desired_angles)

        print "best_desired_angles (rad): %s"%(best_desired_angles)
        print "best_y: %s"%(best_y)
        print "best_n: %s"%(best_n)

        # the normal is the z versor of the new frame C
        Cz_inE = (best_pt[0]-best_pt[1])*(best_pt[0]-best_pt[2])
        if Cz_inE.z() < 0.0:
            Cz_inE = -Cz_inE
        Cx_inE = PyKDL.Vector(1,0,0)
        Cy_inE = Cz_inE * Cx_inE
        Cx_inE = Cy_inE * Cz_inE
        Cx_inE.Normalize()
        Cy_inE.Normalize()
        Cz_inE.Normalize()
        Cp_inE = PyKDL.Vector(x,best_y,((best_pt[0]+best_pt[1]+best_pt[2])*(1.0/3.0)).z())
        T_E_Ge = PyKDL.Frame( PyKDL.Rotation(Cx_inE, Cy_inE, Cz_inE), Cp_inE)

        grasps = []
        for angle in np.arange(0.0, 360.0/180.0*math.pi, 10.0/180.0*math.pi):
            grasps.append(Grasp(copy.deepcopy(T_E_Ge*PyKDL.Frame( PyKDL.Rotation.RotZ(angle))), "JarCapForce",
            [best_desired_angles[0], best_desired_angles[1], best_desired_angles[2], spread_angle],
            [best_desired_angles[0]-20.0/180.0*math.pi, best_desired_angles[1]-20.0/180.0*math.pi, best_desired_angles[2]-20.0/180.0*math.pi, spread_angle]))

        return grasps

    def generateGrasps_jarCapPinch(self, velma, jar):
        spread_angle = 90.0/180.0*numpy.pi

        velma.updateTransformations()

        # we move the jar along y axis i E frame
        # search for such configuration, that every finger touches the jar in the central plane of the finger
        best_n = 1000.0
        x = 0.0
        y = 0.0
        kinematics = [velma.F1_kinematics, velma.F2_kinematics, velma.F3_kinematics]
        finger_index = 0
        contact = [False, False, False]
        desired_angles = [0.0, 0.0, 0.0]
        pt = [PyKDL.Vector(), PyKDL.Vector()]
        T_E_Fd = [PyKDL.Frame(), PyKDL.Frame()]
        # for 2 fingers iterate through joint angles until there is a contact with the jar
        for k in kinematics[0:2]:
            # calculate contact points we want to reach
            for data in k:
                desired_angles[finger_index] = data[0]
                T_E_Fd[finger_index] = velma.get_T_E_Fd(finger_index, data[0], spread_angle)
                # iterate through all cells of the sensor matrix
                for T_F_S in velma.pressure_frames:
                    pt[finger_index] = T_E_Fd[finger_index] * T_F_S * PyKDL.Vector()
                    z = pt[finger_index].z()
                    r = (pt[finger_index]-PyKDL.Vector(x,y,z)).Norm()
                    if r <= jar.R:
                        contact[finger_index] = True
                        break
                if contact[finger_index]:
                    break
            finger_index += 1
        # if we have contact point for each finger, compute the middle point for two points
        if contact[0] and contact[1]:
            center_pt = (pt[0] + pt[1]) * 0.5
        # the normal is the z versor of the new frame C
        Cz_inE = PyKDL.Vector(0,0,1)
        Cx_inE = PyKDL.Vector(1,0,0)
        Cy_inE = Cz_inE * Cx_inE
        Cx_inE = Cy_inE * Cz_inE
        Cx_inE.Normalize()
        Cy_inE.Normalize()
        Cz_inE.Normalize()
        Cp_inE = center_pt
        T_E_Ge = PyKDL.Frame( PyKDL.Rotation(Cx_inE, Cy_inE, Cz_inE), Cp_inE)
        grasps = []
        for angle in np.arange(0.0, 360.0/180.0*math.pi, 10.0/180.0*math.pi):
            grasps.append(Grasp(copy.deepcopy(T_E_Ge*PyKDL.Frame( PyKDL.Rotation.RotZ(angle))), "JarCapPinch",
            [100.0/180.0*math.pi, 100.0/180.0*math.pi, 0.0, spread_angle],
            [0.0, 0.0, 0.0, spread_angle]))
        return grasps

    def GenerateGrasps_jarCapTopTouch(self,velma):
        hand_q = [
        [0, [0.0, 0.0, 0.0, 0.0], [20.0/180.0*math.pi, 0.0, 130.0/180.0*math.pi, 0.0]],
        [0, [0.0, 0.0, 0.0, 90.0/180.0*math.pi], [20.0/180.0*math.pi, 130.0/180.0*math.pi, 0.0, 90.0/180.0*math.pi]],
        [1, [0.0, 0.0, 0.0, 90.0/180.0*math.pi], [130.0/180.0*math.pi, 20.0/180.0*math.pi, 0.0, 90.0/180.0*math.pi]],
        [2, [0.0, 0.0, 0.0, 180.0/180.0*math.pi], [0.0, 0.0, 20.0/180.0*math.pi, 180.0/180.0*math.pi]]
        ]
        velma.updateTransformations()
        # we want to touch the jar's cap with the middle of the 5th row of tactile matrix
        tactile_row = 6
        grasps = []
        for angle in np.arange(0.0, 360.0/180.0*math.pi, 10.0/180.0*math.pi):
            for q in hand_q:
                T_E_Ge = velma.get_T_E_Fd(q[0], q[2][q[0]], q[2][3]) * velma.pressure_frames[tactile_row*3 + 1]
                grasps.append(Grasp(copy.deepcopy(T_E_Ge*PyKDL.Frame( PyKDL.Rotation.RotZ(angle))), "JarCapTopTouch",
                q[2],
                q[1]))
        return grasps

    def poseCallback(self, data):
         print data.pose.position

    def spin(self):

        # create the jar model
        jar = Jar(self.pub_marker)
        # start thread for jar tf publishing and for visualization
        thread.start_new_thread(jar.tfBroadcasterLoop, (0.5, 1))
        # look for jar marker
        T_B_J = self.getJarMarkerPose()
        if T_B_J == None:
            print "Cound not find jar marker."
            exit(0)
        # jar marker is found, add observation to the jar model
        print "Found jar marker."
        jar.addMarkerObservation(T_B_J)

        # calculate angle between jar_axis and vertical vector (z) in B
        jar_up_angle = 180.0*getAngle(PyKDL.Frame(T_B_J.M)*PyKDL.Vector(0,0,1), PyKDL.Vector(0,0,1))/math.pi
        print "angle between jar_axis and vertical vector (z) in B: %s deg."%(jar_up_angle)

        # stiffness for jar touching
        k_jar_touching = Wrench(Vector3(1200.0, 1200.0, 1200.0), Vector3(300.0, 300.0, 300.0))
        k_jar_cap_gripping = Wrench(Vector3(1200.0, 1200.0, 1200.0), Vector3(300.0, 300.0, 300.0))
        k_jar_cap_rotating = Wrench(Vector3(100.0, 100.0, 1200.0), Vector3(300.0, 300.0, 300.0))

        # create the robot interface
        velma = Velma()

        velma.updateTransformations()

        # reset the gripper
        self.resetGripper(velma)

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

        if False:
            velma.moveAroundQ5Singularity(5.0/180.0*math.pi)
            print "aborted_on_q5_singularity: %s"%(velma.aborted_on_q5_singularity)
            exit(0)

        if False:
            velma.updateTransformations()
            i = 6
            omega = 5.0/180.0*math.pi
            if velma.q_r[i] > 0.0:
                angle = -0.0/180.0*math.pi
                omega = -math.fabs(omega)
            else:
                angle = 0.0/180.0*math.pi
                omega = math.fabs(omega)
            traj, times = velma.generateTrajectoryInJoint(i, -velma.q_r[i]+angle, omega)

            velma.moveWrist2(traj[-1]*velma.T_W_T)
            raw_input("Press Enter to move the robot in " + str(times[-1]) + " s...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWristTraj(traj, times, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=False)
            velma.checkStopCondition(times[-1]+0.5)
            exit(0)

        velma.calibrateTactileSensors()
        print "generating grasps..."
        # calculate the best grip for the jar cap
        jarCapForceGrasps = self.generateGrasps_jarCapForce(velma, jar, 70.0/180.0*math.pi)
        print "generated %s grasps: %s"%(len(jarCapForceGrasps), jarCapForceGrasps[0].name)

        # calculate the best grip for the jar side touching
        jarCapPinchGrasps = self.generateGrasps_jarCapPinch(velma, jar)
        print "generated %s grasps: %s"%(len(jarCapPinchGrasps), jarCapPinchGrasps[0].name)

        # calculate the best grip for the jar top touching
        jarCapTopTouchGrasps = self.GenerateGrasps_jarCapTopTouch(velma)
        print "generated %s grasps: %s"%(len(jarCapTopTouchGrasps), jarCapTopTouchGrasps[0].name)

        self.grasps = jarCapForceGrasps + jarCapPinchGrasps + jarCapTopTouchGrasps
        print "Total %s grasps"%(len(self.grasps))

        # get contact points observations
        # we want to touch the jar's cap with the middle of the 5th row of tactile matrix
        if True:
            actions = []
            actions.append(JarLidTopTouchAction(velma))
            actions.append(JarLidPinchGraspAction(velma))
            actions.append(RemoveJarLidAction(velma))
            actions.append(RemoveJarLidAction(velma))
            actions.append(RemoveJarLidAction(velma))
            actions.append(RemoveJarLidAction(velma))
            actions.append(RemoveJarLidAction(velma))

            self.resetContacts()
            jar.resetContactObservations()

            action_index = 0

            last_k = copy.deepcopy(k_jar_touching)
            last_q = [0.0, 0.0, 0.0, 0.0]

            for action in actions:
                # get the fresh pose of the jar
                T_B_JC = copy.deepcopy(jar.getJarCapFrame())
                action.setTarget(T_B_JC)

                stop = False
                action.recalculateRelations(self.grasps)
                rel_index = 0
                while not stop:
                    rel = action.relations[rel_index]
                    print "rel: %s"%(rel.name)

                    if rel.grasp.q_pre != None and (last_q[0] != rel.grasp.q_pre[0] or last_q[1] != rel.grasp.q_pre[1] or last_q[2] != rel.grasp.q_pre[2] or last_q[3] != rel.grasp.q_pre[3]):
                        last_q = copy.deepcopy(rel.grasp.q_pre)
                        velma.move_hand_client("right", rel.grasp.q_pre, t=rel.grasp.t_pre )
                        if velma.checkStopCondition(3.0):
                            exit(0)
                    if rel.k_pre != None and (last_k.force.x != rel.k_pre.force.x or last_k.force.y != rel.k_pre.force.y or last_k.force.z != rel.k_pre.force.z or last_k.torque.x != rel.k_pre.torque.x or last_k.torque.y != rel.k_pre.torque.y or last_k.torque.z != rel.k_pre.torque.z):
                        last_k = copy.deepcopy(rel.k_pre)
                        print "moveImpedance(%s)"%(rel.k_pre)
                        velma.moveImpedance(rel.k_pre, 2.0)
                        if velma.checkStopCondition(2.0):
                            exit(0)

                    T_B_Wd = action.T_B_Go * rel.T_Go_Ge * rel.grasp.T_Ge_E * velma.T_E_W
                    duration = velma.getMovementTime(T_B_Wd, max_v_l = rel.max_v_l, max_v_r = rel.max_v_r)
                    velma.moveWrist2(T_B_Wd*velma.T_W_T)
                    raw_input("Press Enter to move the robot in " + str(duration) + " s...")
                    if velma.checkStopCondition():
                        exit(0)
                    velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                    if True in rel.stop_on_contact:
                        # wait for contact
                        contacts = velma.waitForFirstContact(80, duration, emergency_stop=True, f1=True, f2=True, f3=True, palm=False)
                        if rel.collect_contacts:
                            if len(contacts) < 1:
                                return
                            for c in contacts:
                                self.addContact(c)
#                            print "found contact point"
                    else:
                        if rel.collect_contacts:
                            end_time = rospy.Time.now() + rospy.Duration(duration)
                            while rospy.Time.now() < end_time:
                                contacts = velma.getContactPoints(200, f1=True, f2=True, f3=True, palm=False)
                                for c in contacts:
                                    self.addContact(c)
                                if velma.checkStopCondition(0.01):
                                    exit(0)
                                if velma.aborted_on_q5_singularity:
                                    break
                        else:
                            if velma.checkStopCondition(duration):
                                exit(0)

                    aborted = False
                    if velma.aborted_on_q5_singularity:
                        print "moving away from singularity in q_5..."
                        velma.moveAwayQ5Singularity(20.0/180.0*math.pi, T_B_Wd)
                        aborted = True

                    if velma.aborted_on_q5_q6_self_collision:
                        print "moving away from self collision in q_5 and q_6..."
                        closest_sector = velma.getClosestQ5Q6SpaceSector(velma.q_r[5], velma.q_r[6])
                        print "closest sector: %s"%(closest_sector)
                        q5 = (velma.q5_q6_restricted_area[closest_sector][0]+velma.q5_q6_restricted_area[closest_sector][1])/2.0
                        q6 = (velma.q5_q6_restricted_area[closest_sector][2]+velma.q5_q6_restricted_area[closest_sector][3])/2.0
                        print "q_5: %s   q_6: %s"%(q5,q6)
                        traj, times = velma.generateTrajectoryInQ5Q6(q5,q6,5.0/180.0*math.pi)

                        velma.moveWrist2(traj[-1]*velma.T_W_T)
                        raw_input("Press Enter to move the robot in " + str(times[-1]) + " s...")
                        if velma.checkStopCondition():
                            exit(0)
                        velma.moveWristTraj(traj, times, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=False)
                        velma.checkStopCondition(times[-1]+0.5)
                        aborted = True

                    if aborted:
                        next_actions = rel.on_failure.split(',')
                    else:
                        next_actions = rel.on_success.split(',')
                        if rel.grasp.q != None and (last_q[0] != rel.grasp.q[0] or last_q[1] != rel.grasp.q[1] or last_q[2] != rel.grasp.q[2] or last_q[3] != rel.grasp.q[3]):
                            last_q = copy.deepcopy(rel.grasp.q)
                            velma.move_hand_client("right", rel.grasp.q, t=rel.grasp.t )
                            if rel.collect_contacts:
                                end_time = rospy.Time.now() + rospy.Duration(3.0)
                                while rospy.Time.now() < end_time:
                                    contacts = velma.getContactPoints(200, f1=True, f2=True, f3=True, palm=False)
                                    if velma.checkStopCondition(0.01):
                                        exit(0)
                                    for c in contacts:
                                        self.addContact(c)
                            else:
                                if velma.checkStopCondition(3.0):
                                    exit(0)
                        if rel.k_post != None and (last_k.force.x != rel.k_post.force.x or last_k.force.y != rel.k_post.force.y or last_k.force.z != rel.k_post.force.z or last_k.torque.x != rel.k_post.torque.x or last_k.torque.y != rel.k_post.torque.y or last_k.torque.z != rel.k_post.torque.z):
                            last_k = copy.deepcopy(rel.k_post)
                            print "moveImpedance(%s)"%(rel.k_post)
                            velma.moveImpedance(rel.k_post, 2.0)
                            if velma.checkStopCondition(2.0):
                                exit(0)

                    print "aborted %s   next_actions: %s"%(aborted, next_actions)
                    for next in next_actions:
                        if next == "RECALCULATE":
                            action.recalculateRelations(self.grasps)
                        elif next == "FAILURE":
                            exit(0)
                        elif next == "SUCCESS":
                            if rel.grasp.q != None and (last_q[0] != rel.grasp.q[0] or last_q[1] != rel.grasp.q[1] or last_q[2] != rel.grasp.q[2] or last_q[3] != rel.grasp.q[3]):
                                last_q = copy.deepcopy(rel.grasp.q)
                                velma.move_hand_client("right", rel.grasp.q, t=rel.grasp.t)
                                if rel.collect_contacts:
                                    end_time = rospy.Time.now() + rospy.Duration(3.0)
                                    while rospy.Time.now() < end_time:
                                        contacts = velma.getContactPoints(200, f1=True, f2=True, f3=True, palm=False)
                                        if velma.checkStopCondition(0.01):
                                            exit(0)
                                        for c in contacts:
                                            self.addContact(c)
                                else:
                                    if velma.checkStopCondition(3.0):
                                        exit(0)
                            if rel.k_post != None and (last_k.force.x != rel.k_post.force.x or last_k.force.y != rel.k_post.force.y or last_k.force.z != rel.k_post.force.z or last_k.torque.x != rel.k_post.torque.x or last_k.torque.y != rel.k_post.torque.y or last_k.torque.z != rel.k_post.torque.z):
                                last_k = copy.deepcopy(rel.k_post)
                                velma.moveImpedance(rel.k_post, 2.0)
                                if velma.checkStopCondition(2.0):
                                    exit(0)
                            stop = True
                            break
                        elif action.getRelIndex(next) >= 0:
                            rel_index = action.getRelIndex(next)
                            break
                        else:
                            print "error: could not find relation: %s"%(next)
                            exit(0)

                if action_index == 0:
                    for c in self.contacts:
                        jar.addContactObservation(c)
                    jar.processContactObservationsForTop()
                    jar.drawContactObservations()
                    self.resetContacts()
                    jar.resetContactObservations()
                elif action_index == 1:
                    used_points = [False for c in self.contacts]
                    index = 0
                    for c in self.contacts:
                        if used_points[index]:
                            index += 1
                            continue
                        mean_point = PyKDL.Vector()
                        mean_count = 0
                        index_2 = 0
                        for d in self.contacts:
                            if (c-d).Norm() < 0.02:
                                mean_point += d
                                mean_count += 1
                                used_points[index_2] = True
                            index_2 += 1
                        if mean_count > 0:
                            jar.addContactObservation(mean_point*(1.0/mean_count))
                            
                    # now, we have very good pose of the jar
                    jar.processContactObservations()
                    jar.drawContactObservations()
                else:
                    contacts = velma.getContactPoints(200, f1=True, f2=True, f3=True, palm=False)
                    if len(contacts) > 0:
                        print "the jar is opened"
                        break
                    for c in self.contacts:
                        jar.addContactObservation(c)
                    jar.processContactObservationsForTop(top_offset=-0.02)
                    jar.drawContactObservations()
                    jar.resetContactObservations()

                    used_points = [False for c in self.contacts]
                    index = 0
                    for c in self.contacts:
                        if used_points[index]:
                            index += 1
                            continue
                        mean_point = PyKDL.Vector()
                        mean_count = 0
                        index_2 = 0
                        for d in self.contacts:
                            if (c-d).Norm() < 0.02:
                                mean_point += d
                                mean_count += 1
                                used_points[index_2] = True
                            index_2 += 1
                        if mean_count > 0:
                            jar.addContactObservation(mean_point*(1.0/mean_count))
                            
                    # now, we have very good pose of the jar
                    jar.processContactObservations()


#                jar.drawContactObservations()

                action_index += 1
#                jar.drawContactObservations()

            rospy.sleep(2.0)

            exit(0)

if __name__ == '__main__':

    rospy.init_node('jar_opener')

    global br
    pub_marker = MarkerPublisher()
    task = JarOpener(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()
    



