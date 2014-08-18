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

def publishSinglePointMarker(pt, i, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005)):
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
    pub_marker.publish(m)

def publishMultiPointsMarker(pt, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002)):
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
    pub_marker.publish(m)

def publishVectorMarker(v1, v2, i, r, g, b, frame='torso_base', namespace='default'):
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
    marker.scale = Vector3(0.001, 0.002, 0)
    marker.color = ColorRGBA(r,g,b,0.5)
    m.markers.append(marker)

    pub_marker.publish(m)

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
        R_count = int( self.R/distance )
        for r in range(0, R_count+1):
            current_r = (float(r)/R_count)*self.R
            L = 2.0*math.pi*current_r
            L_count = int( L/distance ) + 1
            for l in range(0, L_count):
               angle = 2.0*math.pi*float(l)/L_count
               self.pt.append(PyKDL.Vector(current_r*math.cos(angle), current_r*math.sin(angle), 0.0)) 
               self.pt.append(PyKDL.Vector(current_r*math.cos(angle), current_r*math.sin(angle), self.H)) 

    def __init__(self):
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
        publishMultiPointsMarker(self.pt, 0, 1, 0, namespace="jar_points", frame_id="jar")

    def publishTf(self):
        pose = pm.toMsg(self.T_B_Jbase)
        br.sendTransform([pose.position.x, pose.position.y, pose.position.z], [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], rospy.Time.now(), "jar", "torso_base")

    def tfBroadcasterLoop(self, interval, *args):
        while not rospy.is_shutdown():
            self.publishTf()
            self.drawJar()
            rospy.sleep(interval)

    def drawJar(self):
        publishSinglePointMarker(PyKDL.Vector(0,0,self.H*0.5), 0, r=0, g=1, b=0, namespace='jar', frame_id='jar', m_type=Marker.CYLINDER, scale=Vector3(self.R*2.0, self.R*2.0, self.H))

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
        publishMultiPointsMarker(self.contacts_Jbase, 1, 0, 0, namespace="jar_obs", frame_id="jar")

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

    def processContactObservationsForTop(self):
        if len(self.contacts_Jbase) < 1:
            return
        max_z = 0.0
        for i in range(0, len(self.contacts_Jbase)):
            if self.contacts_Jbase[i].z() > max_z:
                max_z = self.contacts_Jbase[i].z()
        position = PyKDL.Vector(0,0,self.H-max_z)
#        print "processContactObservationsForTop: position: %s"%(position)
        for i in range(0, len(self.contacts_Jbase)):
#            print "processContactObservationsForTop: self.contacts_Jbase[%s]: %s"%(i,self.contacts_Jbase[i])
            self.contacts_Jbase[i] += position
#            print "processContactObservationsForTop: self.contacts_Jbase[%s]: %s"%(i,self.contacts_Jbase[i])
        self.T_B_Jbase = copy.deepcopy( self.T_B_Jbase * PyKDL.Frame(-position) )

    def getJarCapFrame(self):
        return self.T_B_Jbase * self.T_Jbase_Jmarker

class JarOpener:
    """
Class for opening the jar.
"""

    def __init__(self):
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
#        self.q_start = (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
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
            publishSinglePointMarker(pt, 0, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))

            T_E_Fd = robot.get_T_E_Fd(1, math.pi*angle/180.0)
            pt = robot.T_B_W * robot.T_W_E * T_E_Fd * PyKDL.Vector(0.05, -0.01, 0)
            publishSinglePointMarker(pt, 1, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))

            T_E_Fd = robot.get_T_E_Fd(2, math.pi*angle/180.0)
            pt = robot.T_B_W * robot.T_W_E * T_E_Fd * PyKDL.Vector(0.05, -0.01, 0)
            publishSinglePointMarker(pt, 2, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))
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

    def spin(self):

        # create the jar model
        jar = Jar()
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
        if jar_up_angle > 30.0:
            print "the jar angle is too big"
            exit(0)

        # test for contact points observation
        if False:
            for i in range(0, 40):
                jar.addContactObservation(jar.T_B_Jbase*(jar.pt[random.randint(0,len(jar.pt)-1)]+PyKDL.Vector(0.05,0,0)))
            jar.drawContactObservations()
            rospy.sleep(2)
            jar.processContactObservations()
            rospy.sleep(2)
            exit(0)

        # stiffness for jar gripping
        k_jar = Wrench(Vector3(800.0, 800.0, 800.0), Vector3(200.0, 200.0, 200.0))

        # create the robot interface
        velma = Velma()

        velma.updateTransformations()

        # start with very low stiffness
        print "setting stiffness to very low value"
        velma.moveImpedance(velma.k_error, 0.5)
        if velma.checkStopCondition(0.5):
            exit(0)

        raw_input("Press Enter to continue...")
        if velma.checkStopCondition():
            exit(0)

        velma.updateAndMoveTool( PyKDL.Frame(PyKDL.Vector(0.2,0,-0.05)), 2.0 )
        if velma.checkStopCondition(2.0):
            exit(0)

        raw_input("Press Enter to continue...")
        print "setting stiffness to bigger value"

        velma.moveImpedance(k_jar, 5.0)
        if velma.checkStopCondition(5.0):
            exit(0)

        # test hand kinematics
        #self.testHandKinematics(velma)

        # collect gripper kinematics
        #self.collectGripperKinematics(velma)

        # test for real contact points observation
        if True:

            # reset the gripper
            self.resetGripper(velma)

            # prepare hook gripper configuration
            velma.move_hand_client("right", (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 180.0/180.0*numpy.pi) )
            if velma.checkStopCondition(1.0):
                exit(0)

            # set hook gripper configuration
            velma.move_hand_client("right", (40.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 180.0/180.0*numpy.pi) )
            if velma.checkStopCondition(2.0):
                exit(0)

            # go to jar approach pose (45 deg.)
            velma.updateTransformations()

            R_B_Ed = PyKDL.Rotation.RotZ(45.0/180.0*math.pi) * PyKDL.Rotation.RotY((90.0+45.0)/180.0*math.pi)
            T_B_Wd = velma.calculateMoveGripperPointToPose( velma.T_E_F*PyKDL.Vector(0.05,-0.01,0), R_B_Ed, T_B_J*PyKDL.Vector(0,0,0.05) )# + PyKDL.Vector(0,0,0.05) )
            velma.moveWrist(T_B_Wd, 6.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(6.0):
                exit(0)

            # rotate the gripper to prepare for jar cap touching
            R_B_Ed = PyKDL.Rotation.RotZ(45.0/180.0*math.pi) * PyKDL.Rotation.RotY((90.0+45.0)/180.0*math.pi) * PyKDL.Rotation.RotZ(-90.0/180.0*math.pi)

            T_B_Wd = velma.calculateMoveGripperPointToPose( velma.T_E_F*PyKDL.Vector(0.05,-0.01,0), R_B_Ed, T_B_J*PyKDL.Vector(0, 0, 0.05) )# + PyKDL.Vector(0,0,0.05) )
            velma.moveWrist(T_B_Wd, 5.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(5.0):
                exit(0)

            # move down
            self.resetContacts()

            T_B_Wd = velma.calculateMoveGripperPointToPose( velma.T_E_F*PyKDL.Vector(0.05,-0.01,0), R_B_Ed, T_B_J*PyKDL.Vector(0, 0, -0.05) )# + PyKDL.Vector(0,0,-0.05) )
            velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            # wait for contact
            contacts = velma.waitForFirstContact(50, 11.0, emergency_stop=True)
            if len(contacts) < 1:
                return
            for c in contacts:
                self.addContact(c)

            print "found contact point"
            for c in self.contacts:
                jar.addContactObservation(c)
            jar.drawContactObservations()
            raw_input("Press Enter to continue...")

            # update jar position
            print T_B_J
            jar.processContactObservationsForTop()
            rospy.sleep(1.0)
            jar.drawContactObservations()
            T_B_J = copy.deepcopy(jar.getJarCapFrame())
            print T_B_J
            raw_input("Press Enter to continue...")

            # move up
            T_B_Wd = velma.calculateMoveGripperPointToPose( velma.T_E_F*PyKDL.Vector(0.05,-0.01,0), R_B_Ed, T_B_J*PyKDL.Vector(0, 0, 0.05) )# + PyKDL.Vector(0,0,0.05) )
            velma.moveWrist(T_B_Wd, 2.5, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(2.5):
                exit(0)

            # go to jar approach pose (45 deg.)
            R_B_Ed = PyKDL.Rotation.RotZ(45.0/180.0*math.pi) * PyKDL.Rotation.RotY((90.0+45.0)/180.0*math.pi)
            T_B_Wd = velma.calculateMoveGripperPointToPose( PyKDL.Vector(0,0,(velma.T_E_F*PyKDL.Vector(0.05,-0.01,0)).z()), R_B_Ed, T_B_J*PyKDL.Vector(0, 0, 0.05) )# + PyKDL.Vector(0,0,0.05) )
            velma.moveWrist(T_B_Wd, 5.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            # set cylindrical gripper configuration
            velma.move_hand_client("right", (40.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 0.0/180.0*numpy.pi) )
            if velma.checkStopCondition(5.0):
                exit(0)

            velma.updateTransformations()

            pt1 = velma.T_E_F13 * PyKDL.Vector(0.03,-0.01,0)
            pt2 = velma.T_E_F33 * PyKDL.Vector(0.03,-0.01,0)
            print pt1
            print pt2
#            params = [[20.0, pt1], [20.0, pt2], [45.0, pt1], [45.0, pt2], [70.0, pt1], [70.0, pt2]]
            params = [[20.0, pt1], [20.0, pt2], [70.0, pt1], [70.0, pt2]]

            for param in params:
                # go to jar approach pose (45 deg.)
                R_B_Ed = PyKDL.Rotation.RotZ(param[0]/180.0*math.pi) * PyKDL.Rotation.RotY((90.0+45.0)/180.0*math.pi)
                T_B_Wd = velma.calculateMoveGripperPointToPose( 0.5*(pt1+pt2), R_B_Ed, T_B_J*PyKDL.Vector() )

                velma.moveWrist(T_B_Wd, 2.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                if velma.checkStopCondition(2.0):
                    exit(0)

                # search contact
                R_B_Ed = PyKDL.Rotation.RotZ(param[0]/180.0*math.pi) * PyKDL.Rotation.RotY((90.0+45.0)/180.0*math.pi)
                T_B_Wd = velma.calculateMoveGripperPointToPose( param[1], R_B_Ed, T_B_J*PyKDL.Vector() )
                velma.moveWrist(T_B_Wd, 5, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                # wait for contact
                contacts = velma.waitForFirstContact(50, 5.0, emergency_stop=True)
                if len(contacts) < 1:
                    return
                for c in contacts:
                    self.addContact(c)

                # go to jar approach pose (45 deg.)
                R_B_Ed = PyKDL.Rotation.RotZ(param[0]/180.0*math.pi) * PyKDL.Rotation.RotY((90.0+45.0)/180.0*math.pi)
                T_B_Wd = velma.calculateMoveGripperPointToPose( 0.5*(pt1+pt2), R_B_Ed, T_B_J*PyKDL.Vector() )

                velma.moveWrist(T_B_Wd, 2, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                if velma.checkStopCondition(2):
                    exit(0)

            # move to center
#            R_B_Ed = PyKDL.Rotation.RotZ(45.0/180.0*math.pi) * PyKDL.Rotation.RotY((90.0+45.0)/180.0*math.pi)
#            T_B_Wd = velma.calculateMoveGripperPointToPose( PyKDL.Vector(0,0,(velma.T_E_F*PyKDL.Vector(0.05,0.0,0)).z()), R_B_Ed, T_B_J*PyKDL.Vector() )
#            velma.moveWrist(T_B_Wd, 2.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
#            if velma.checkStopCondition(2.0):
#                exit(0)

            for c in self.contacts:
                jar.addContactObservation(c)
            jar.drawContactObservations()
            raw_input("Press Enter to continue...")
            rospy.sleep(2.0)
            jar.processContactObservations()
            T_B_J = copy.deepcopy(jar.getJarCapFrame())
            print T_B_J
            rospy.sleep(1.0)
            jar.drawContactObservations()
            rospy.sleep(2)

#            velma.emergencyStop()

#            exit(0)

#            print "move the gripper aroung and touch the jar"
#            print "connecting contact points for 20s..."
#            self.resetContacts()
#            end_time = rospy.Time.now() + rospy.Duration(20.0)
#            while rospy.Time.now() < end_time:
#                contacts = velma.getContactPoints(100)
#                for c in contacts:
#                    self.addContact(c)
#            for c in self.contacts:
#                jar.addContactObservation(c)
#            jar.drawContactObservations()
#            rospy.sleep(2)
#            jar.processContactObservations()
#            rospy.sleep(2)

#            exit(0)

        if False:
            velma.updateTransformations()

            # move gripper above the jar
            # calculate desired orientation of wrist in frame B
            T_B_Wd = PyKDL.Frame( PyKDL.Rotation.RotZ(math.pi/2.0) )
            # calculate desired orientation of gripper in frame B
            T_B_Ed = T_B_Wd * velma.T_W_E

            T_B_Wd = velma.calculateMoveGripperPointToPose( PyKDL.Vector(0,0,0), T_B_Ed.M, T_B_J * PyKDL.Vector(0,0,0.2) )
            velma.moveWrist2(T_B_Wd)
            raw_input("Press Enter to continue...")
            if velma.checkStopCondition():
                exit(0)

            velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(10.0):
                exit(0)

            # rotate gripper down
            velma.updateTransformations()
            T_E_Ed = PyKDL.Frame( PyKDL.Rotation.RotY(math.pi/2.0) )
            T_B_Wd = velma.T_B_W * velma.T_W_E * T_E_Ed * velma.T_E_W

            velma.moveWrist2(T_B_Wd)
            raw_input("Press Enter to continue...")
            if velma.checkStopCondition():
                exit(0)

            velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(10.0):
                exit(0)

        raw_input("Press Enter to continue...")

        # set gripper configuration for jar decap
        velma.move_hand_client("right", (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi) )
        if velma.checkStopCondition(3.0):
            exit(0)
        velma.updateTransformations()

        jar_cap = PyKDL.Vector()
        desired_angles = []
        desired_points_B = []
        desired_points_E = []
        kinematics = [velma.F1_kinematics, velma.F2_kinematics, velma.F3_kinematics]

        finger = 0
        for k in kinematics:
            # calculate contact points we want to reach
            for data in k:
                T_E_Fd = velma.get_T_E_Fd(finger, data[0])
#                pt_E = T_E_Fd * PyKDL.Vector(0.0510813, -0.0071884, 0)
                pt_E = T_E_Fd * velma.pressure_frames[16] * PyKDL.Vector()
                diff = pt_E-jar_cap
                diff = PyKDL.Vector(diff.x(), diff.y(), 0.0)
                if diff.Norm() < jar.R-0.005:
                    publishSinglePointMarker(pt_E, finger, r=0, g=1, b=0, namespace='default', frame_id='right_HandPalmLink', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))
                    desired_angles.append(data[0])
                    desired_points_B.append(copy.deepcopy(velma.T_B_W * velma.T_W_E * pt_E))
                    desired_points_E.append(copy.deepcopy(pt_E))
                    break
            finger += 1
        print "desired_angles: %s"%(desired_angles)
        print "desired_points_B: %s"%(desired_points_B)

        velma.updateTransformations()

        # calculate plane normal from desired contact points in E
        contacts_normal_E = (desired_points_E[0]-desired_points_E[1])*(desired_points_E[0]-desired_points_E[2])
        # make sure the normal is directed outside the gripper (E.z)
        if contacts_normal_E.z() < 0:
            contacts_normal_E = -contacts_normal_E

        # the normal is the z versor of the new frame C
        Cz_inE = contacts_normal_E
        Cx_inE = PyKDL.Vector(1,0,0)
        Cy_inE = Cz_inE * Cx_inE
        Cx_inE = Cy_inE * Cz_inE
        Cx_inE.Normalize()
        Cy_inE.Normalize()
        Cz_inE.Normalize()
        Cp_inE = PyKDL.Vector(0,0,((desired_points_E[0]+desired_points_E[1]+desired_points_E[2])*(1.0/3.0)).z())
        T_E_C = PyKDL.Frame( PyKDL.Rotation(Cx_inE, Cy_inE, Cz_inE), Cp_inE)

#        publishVectorMarker(Cp_inE, Cp_inE+Cx_inE*0.1, 0, 1, 0, 0, frame='right_HandPalmLink', namespace='C_frame')
#        publishVectorMarker(Cp_inE, Cp_inE+Cy_inE*0.1, 1, 0, 1, 0, frame='right_HandPalmLink', namespace='C_frame')
#        publishVectorMarker(Cp_inE, Cp_inE+Cz_inE*0.1, 2, 0, 0, 1, frame='right_HandPalmLink', namespace='C_frame')

        # calculate contact frame for jar
        v = (T_B_J * PyKDL.Vector()) - (velma.T_B_L2 * PyKDL.Vector())

        T_B_Cd_x = v * (PyKDL.Frame(T_B_J.M)*PyKDL.Vector(0,0,1))
        T_B_Cd_z = -(PyKDL.Frame(T_B_J.M)*PyKDL.Vector(0,0,1))
        T_B_Cd_y = T_B_Cd_z * T_B_Cd_x
        T_B_Cd_x.Normalize()
        T_B_Cd_y.Normalize()
        T_B_Cd_z.Normalize()
        T_B_Cd_p = T_B_J * PyKDL.Vector()
        T_B_Cd = PyKDL.Frame( PyKDL.Rotation(T_B_Cd_x, T_B_Cd_y, T_B_Cd_z), T_B_Cd_p)

        publishVectorMarker(T_B_Cd_p, T_B_Cd_p+T_B_Cd_x*0.1, 3, 1, 0, 0, frame='torso_base', namespace='C_frame')
        publishVectorMarker(T_B_Cd_p, T_B_Cd_p+T_B_Cd_y*0.1, 4, 0, 1, 0, frame='torso_base', namespace='C_frame')
        publishVectorMarker(T_B_Cd_p, T_B_Cd_p+T_B_Cd_z*0.1, 5, 0, 0, 1, frame='torso_base', namespace='C_frame')

        T_B_Ed = T_B_Cd * T_E_C.Inverse()

        # go to position above the jar cap
        T_B_Wd = velma.calculateMoveGripperPointToPose( T_E_C*PyKDL.Vector(), T_B_Ed.M, T_B_J * PyKDL.Vector(0,0,0.10) )
        velma.moveWrist2(T_B_Wd * velma.T_W_T)
        raw_input("Press Enter to continue...")
        if velma.checkStopCondition():
            exit(0)
        velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if velma.checkStopCondition(10.0):
            exit(0)

        # correct gripper configuration for jar decap
        velma.move_hand_client("right", (desired_angles[0] - 20.0/180.0*numpy.pi, desired_angles[1] - 20.0/180.0*numpy.pi, desired_angles[2] - 20.0/180.0*numpy.pi, 0.0/180.0*numpy.pi) )
        if velma.checkStopCondition(3.0):
            exit(0)

        # go to position for jar decap
        T_B_Wd = velma.calculateMoveGripperPointToPose( T_E_C*PyKDL.Vector(), T_B_Ed.M, T_B_J * PyKDL.Vector(0,0,-0.01) )
        velma.moveWrist2(T_B_Wd * velma.T_W_T)
        raw_input("Press Enter to continue...")
        if velma.checkStopCondition():
            exit(0)
        velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if velma.checkStopCondition(10.0):
            exit(0)

        raw_input("Press Enter to continue...")

        # close the fingers on the cap
        velma.move_hand_client("right", (desired_angles[0] + 10.0/180.0*numpy.pi, desired_angles[1] + 10.0/180.0*numpy.pi, desired_angles[2] + 10.0/180.0*numpy.pi, 0.0/180.0*numpy.pi), t=(1000, 1000, 1000, 1000) )
        if velma.checkStopCondition(3.0):
            exit(0)

        # loop for jar decap
        # initial cap angle
        cap_angle = 0.0
        # start joint_states listener to get joint position and check if it is near limit
        self.q5 = None
        joint_states_listener = rospy.Subscriber('/joint_states', JointState, self.jointStatesCallback)
        rospy.sleep(1.0)
        # -1.8: near limit
        # -0.8: near singularity
        q5_singularity = -0.8
        q5_bottom_limit = -1.8
        angle = 0.0
        pos_z = -0.01
        while True:#cap_angle < math.pi*180.0/180.0:

            angle_prev = angle
            # rotate the cap
            while True:
                self.joint_states_lock.acquire()
                q5 = copy.copy(self.q5)
                self.joint_states_lock.release()

                if q5 == None:
                    print "could not determine angle in joint 5"
                    exit(0)

                print "q5: %s"%(q5)

                if q5 < q5_bottom_limit:
                    break

                angle -= 0.02
                print "angle: %s deg."%(180.0*angle/math.pi)
                T_B_Ed2 = PyKDL.Frame(PyKDL.Rotation.Rot(PyKDL.Frame(T_B_Cd.M)*PyKDL.Vector(0,0,1), angle)) * T_B_Ed
                T_B_Wd = velma.calculateMoveGripperPointToPose( T_E_C*PyKDL.Vector(), T_B_Ed2.M, T_B_J * PyKDL.Vector(0,0,pos_z) )
#                velma.moveWrist2(T_B_Wd * velma.T_W_T)
#                raw_input("Press Enter to continue...")
#                if velma.checkStopCondition():
#                    exit(0)
                velma.moveWrist(T_B_Wd, 0.1, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                if velma.checkStopCondition(0.1):
                    exit(0)

            # pull the cap
            # first, check the current position of the gripper in jar frame
            velma.updateTransformations()
#            pt_C_in_B = velma.T_B_W * velma.T_W_E * T_E_C * PyKDL.Vector()
#            pt_C_in_J = T_B_J.Inverse() * pt_C_in_B
#            old_z = pt_C_in_J.z()
            # pull the cap in the J.z direction by 3cm
            T_B_Wd = velma.calculateMoveGripperPointToPose( T_E_C*PyKDL.Vector(), T_B_Ed2.M, T_B_J * PyKDL.Vector(0,0,pos_z+0.05) )
            self.resetContacts()
            velma.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            end_t = rospy.Time.now() + rospy.Duration(3.0)
            while rospy.Time.now() < end_t:
                contacts = velma.getContactPoints(200)
                if len(contacts) < 1:
                    break
                for c in contacts:
                    self.addContact(c)

            if len(self.contacts) < 1:
                print "no contact with the jar"
                break
            max_z = 0.0
            for c in self.contacts:
                c_in_J = T_B_J.Inverse() * c
                if c_in_J.z() > max_z:
                    max_z = c_in_J.z()

            pos_z = max_z - 0.01
            print "max_z: %s"%(max_z)
            print "pos_z: %s"%(pos_z)
            raw_input("Press Enter to continue...")
            if velma.checkStopCondition():
                exit(0)


            cap_angle -= angle - angle_prev

            # open the fingers
            velma.move_hand_client("right", (desired_angles[0] - 20.0/180.0*numpy.pi, desired_angles[1] - 20.0/180.0*numpy.pi, desired_angles[2] - 20.0/180.0*numpy.pi, 0.0/180.0*numpy.pi) )
            if velma.checkStopCondition(2.0):
                exit(0)

            T_B_Wd = velma.calculateMoveGripperPointToPose( T_E_C*PyKDL.Vector(), T_B_Ed2.M, T_B_J * PyKDL.Vector(0,0,pos_z) )
            velma.moveWrist(T_B_Wd, 1.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(1.0):
                exit(0)

            while True:
                velma.updateTransformations()

                angle_link_5_7 = getAngle( PyKDL.Frame(velma.T_B_L5.M)*PyKDL.Vector(0,0,1), PyKDL.Frame(velma.T_B_L7.M)*PyKDL.Vector(0,0,1) )
                print "angle_link_5_7: %s deg."%(180.0*angle_link_5_7/math.pi)

                if angle_link_5_7 < math.pi*30.0/180.0:
                    break

                angle += 0.04
                print "angle: %s deg."%(180.0*angle/math.pi)
                T_B_Ed2 = PyKDL.Frame(PyKDL.Rotation.Rot(PyKDL.Frame(T_B_Cd.M)*PyKDL.Vector(0,0,1), angle)) * T_B_Ed
                T_B_Wd = velma.calculateMoveGripperPointToPose( T_E_C*PyKDL.Vector(), T_B_Ed2.M, T_B_J * PyKDL.Vector(0,0,pos_z) )
                velma.moveWrist(T_B_Wd, 0.1, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                if velma.checkStopCondition(0.1):
                    exit(0)

            # close the fingers on the cap
            velma.move_hand_client("right", (desired_angles[0] + 10.0/180.0*numpy.pi, desired_angles[1] + 10.0/180.0*numpy.pi, desired_angles[2] + 10.0/180.0*numpy.pi, 0.0/180.0*numpy.pi), t=(1000, 1000, 1000, 1000) )
            if velma.checkStopCondition(2.0):
                exit(0)


        joint_states_listener.unregister()

        rospy.sleep(2)
        exit(0)

        print "self.q5: %s"%(self.q5)
        T_L6_L6d = PyKDL.Frame(PyKDL.Rotation.RotY(q5_near_singularity-self.q5))
        T_B_Ed = velma.T_B_L6 * T_L6_L6d * velma.T_B_L6.Inverse() * velma.T_B_W * velma.T_W_E

        T_B_Wd = velma.calculateMoveGripperPointToPose( PyKDL.Vector(0,0,mean_z_E), T_B_Ed.M, T_B_J * PyKDL.Vector(0,0,0.05) )
        velma.moveWrist2(T_B_Wd * velma.T_W_T)
        raw_input("Press Enter to continue...")
        if velma.checkStopCondition():
                exit(0)

        velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if velma.checkStopCondition(10.0):
            exit(0)


if __name__ == '__main__':

    rospy.init_node('jar_opener')

    global pub_marker
    global br
    pub_marker = rospy.Publisher('/door_markers', MarkerArray)
    task = JarOpener()
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()
    



