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
        return i+1

    def publishMultiPointsMarker(self, pt, base_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=None):
        m = MarkerArray()
        ret_id = copy.copy(base_id)
        for i in range(0, len(pt)):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = ret_id
            ret_id += 1
            marker.type = m_type
            marker.action = 0
            if T != None:
                point = T*pt[i]
                marker.pose = Pose( Point(point.x(),point.y(),point.z()), Quaternion(0,0,0,1) )
            else:
                marker.pose = Pose( Point(pt[i].x(),pt[i].y(),pt[i].z()), Quaternion(0,0,0,1) )
            marker.scale = scale
            marker.color = ColorRGBA(r,g,b,0.5)
            m.markers.append(marker)
        self.pub_marker.publish(m)
        return ret_id

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

class OpenraveInstance:

    def __init__(self, robot, T_World_Br):
        self.robot = robot
        self.rolling = False
        self.env = None
        self.T_World_Br = T_World_Br
        self.listener = tf.TransformListener();

    def convertTransform(self, T):
        ret = numpy.array([
        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
        [0, 0, 0, 1]])
        return ret

    def convertToKDL(self, T):
        rot = PyKDL.Rotation(T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2])
        pos = PyKDL.Vector(T[0][3], T[1][3], T[2][3])
#        ret = numpy.array([
#        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
#        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
#        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
#        [0, 0, 0, 1]])
        return PyKDL.Frame(rot, pos)

    def addBox(self, name, x_size, y_size, z_size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromBoxes(numpy.array([[0,0,0,0.5*x_size,0.5*y_size,0.5*z_size]]),True)
        self.env.Add(body,True)

    def updatePose(self, name, T_Br_Bo):
        body = self.env.GetKinBody(name)
        if body != None:
            body.SetTransform(self.convertTransform(self.T_World_Br*T_Br_Bo))

    def getPose(self, name):
        body = self.env.GetKinBody(name)
        if body != None:
            return self.T_World_Br.Inverse() * self.convertToKDL(body.GetTransform())
        return None

    def main(self,env,options):
        try:
            self.env = env
            self.robot_rave = env.ReadRobotXMLFile('robots/velma_col.robot.xml')

            arms_joint_names = [
            "left_arm_0_joint",
            "left_arm_1_joint",
            "left_arm_2_joint",
            "left_arm_3_joint",
            "left_arm_4_joint",
            "left_arm_5_joint",
            "left_arm_6_joint",
            "right_arm_0_joint",
            "right_arm_1_joint",
            "right_arm_2_joint",
            "right_arm_3_joint",
            "right_arm_4_joint",
            "right_arm_5_joint",
            "right_arm_6_joint",
            ]

            for j in self.robot_rave.GetJoints():
                print j

            # apply soft limits
            q_soft_limit = 0.26
            for name in arms_joint_names:
                joint = self.robot_rave.GetJoint(name)
                lower, upper = joint.GetLimits()
                lower[0] += q_soft_limit
                upper[0] -= q_soft_limit
                joint.SetLimits(lower, upper)

            # apply limits for arms
#            joint = self.robot_rave.GetJoint("left_arm_1_joint")
#            lower, upper = joint.GetLimits()
#            upper[0] = -10.0/180.0*math.pi
#            joint.SetLimits(lower, upper)
#            joint = self.robot_rave.GetJoint("right_arm_1_joint")
#            lower, upper = joint.GetLimits()
#            lower[0] = 10.0/180.0*math.pi
#            joint.SetLimits(lower, upper)

            # apply limits for elbows
            joint = self.robot_rave.GetJoint("left_arm_3_joint")
            lower, upper = joint.GetLimits()
            upper[0] = -10.0/180.0*math.pi
            joint.SetLimits(lower, upper)
            joint = self.robot_rave.GetJoint("right_arm_3_joint")
            lower, upper = joint.GetLimits()
            lower[0] = 10.0/180.0*math.pi
            joint.SetLimits(lower, upper)
            
            # apply limits for wrists
            joint = self.robot_rave.GetJoint("left_arm_5_joint")
            lower, upper = joint.GetLimits()
            lower[0] = 20.0/180.0*math.pi
            joint.SetLimits(lower, upper)
            joint = self.robot_rave.GetJoint("right_arm_5_joint")
            lower, upper = joint.GetLimits()
            print lower, upper
            upper[0] = -20.0/180.0*math.pi
            joint.SetLimits(lower, upper)

            env.Add(self.robot_rave)

            joint = self.robot_rave.GetJoint("right_arm_5_joint")
            lower, upper = joint.GetLimits()
            print lower, upper

#            print "reading gripper..."
#            self.gripper_rave = env.ReadRobotXMLFile('robots/barretthand_col.robot.xml')
#            print "adding gripper..."
#            env.Add(self.gripper_rave)
#            print "gripper ok"

            self.robot_rave.SetActiveManipulator('right_arm')

            ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot_rave,iktype=IkParameterizationType.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()

            links = self.robot_rave.GetLinks()
            # 31   <link:right_HandPalmLink (31), parent=Velma>
#            for i in range(0, 40):
#                print "%s   %s"%(i, links[i])

#            cam_pos = PyKDL.Vector(0.5, 0.0, 0.2)
#            target_pos = PyKDL.Vector(0.0, 0.0, 0.0)
            cam_pos = PyKDL.Vector(2.0, 0.0, 2.0)
            target_pos = PyKDL.Vector(0.60, 0.0, 1.10)

            cam_z = target_pos - cam_pos
            focalDistance = cam_z.Norm()
            cam_y = PyKDL.Vector(0,0,-1)
            cam_x = cam_y * cam_z
            cam_y = cam_z * cam_x
            cam_x.Normalize()
            cam_y.Normalize()
            cam_z.Normalize()
            cam_T = PyKDL.Frame(PyKDL.Rotation(cam_x,cam_y,cam_z), cam_pos)
            
            env.GetViewer().SetCamera(self.convertTransform(cam_T), focalDistance)

            while not rospy.is_shutdown():
                self.rolling = True
                q_lf = [self.robot.q_lf[0], self.robot.q_lf[1], self.robot.q_lf[4], self.robot.q_lf[6]]
                q_rf = [self.robot.q_rf[0], self.robot.q_rf[1], self.robot.q_rf[4], self.robot.q_rf[6]]
                dof_values = self.robot.q_t + self.robot.q_l + q_lf + self.robot.q_r + q_rf
                self.robot_rave.SetDOFValues(dof_values)

                rospy.sleep(0.1)
        finally:
            self.rolling = False
            env.Destroy()
            #RaveDestroy()

    def run(self, args, *args2):
        parser = OptionParser(description='Openrave Velma interface')
        OpenRAVEGlobalArguments.addOptions(parser)
        (options, leftargs) = parser.parse_args(args=args)
        OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,self.main,defaultviewer=True)

    def startNewThread(self):
        # start thread for jar tf publishing and for visualization
        thread.start_new_thread(self.run, (None,1))

    def generateGrasps(self, target_name, show=False):
        target = self.env.GetKinBody(target_name)
        if target == None:
            print "target body <%s> not found"%(target_name)
            return False

        self.gmodel = databases.grasping.GraspingModel(self.robot_rave,target)
        if not self.gmodel.load():
            print 'generating grasping model (one time computation)'
            self.gmodel.init(friction=1.0,avoidlinks=[])
            approachrays = self.gmodel.computeBoxApproachRays(delta=0.05,normalanglerange=0.0)#201, directiondelta=0.2)
            print len(approachrays)
# possible arguments for generate:
# preshapes=None, standoffs=None, rolls=None, approachrays=None, graspingnoise=None, forceclosure=True, forceclosurethreshold=1.0000000000000001e-09, checkgraspfn=None, manipulatordirections=None, translationstepmult=None, finestep=None, friction=None, avoidlinks=None, plannername=None, boxdelta=None, spheredelta=None, normalanglerange=None
# http://openrave.org/docs/latest_stable/openravepy/databases.grasping/#openravepy.databases.grasping.GraspingModel.generatepcg
            self.gmodel.generate(approachrays=approachrays, forceclosure=False, standoffs=[0.025, 0.05, 0.075])
            self.gmodel.save()

        validgrasps,validindices = self.gmodel.computeValidGrasps(checkcollision=True, checkik=True, checkgrasper=True)
        print "all valid grasps: %s"%(len(validgrasps))
#        for validgrasp in validgrasps:
#            print validgrasp
#            self.gmodel.showgrasp(validgrasp, collisionfree=True, useik=True)
#            with RobotStateSaver(self.robot_rave):
#                with self.env:
#                    # check if grasp can be reached by robot
#                    Tglobalgrasp = self.gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
#                    # have to set the preshape since the current robot is at the final grasp!
#                    self.gmodel.setPreshape(validgrasp)
#                    sol = self.gmodel.manip.FindIKSolution(Tglobalgrasp,True)
#                    print sol

        print "done."
        return validgrasps,validindices

    def getGraspTransform(self, grasp, collisionfree=False):
        return self.T_World_Br.Inverse()*self.convertToKDL( self.gmodel.getGlobalGraspTransform(grasp,collisionfree=collisionfree) )

    def showGrasp(self, grasp):
        self.gmodel.showgrasp(grasp, collisionfree=True, useik=True)

    def getGraspStandoff(self, grasp):
        return grasp[self.gmodel.graspindices.get('igraspstandoff')]

    def showTrajectory(self, T_B_Ed, time):
        with RobotStateSaver(self.robot_rave):
            with self.env:
                steps = time/0.1
                if steps < 3:
                    steps = 3
                time_set = np.linspace(0.0, 1.0, steps)
                for t in time_set:
                    #(T_B_Ed, progress, q_start_in=None, q_end_out=None, T_B_Einit=None):
                    q_end = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    self.robot.simulateTrajectory(T_B_Ed, t, q_end_out=q_end)
                    q_lf = [self.robot.q_lf[0], self.robot.q_lf[1], self.robot.q_lf[4], self.robot.q_lf[6]]
                    q_rf = [self.robot.q_rf[0], self.robot.q_rf[1], self.robot.q_rf[4], self.robot.q_rf[6]]
                    dof_values = self.robot.q_t + self.robot.q_l + q_lf + q_end + q_rf
                    self.robot_rave.GetController().Reset(0)
                    self.robot_rave.SetDOFValues(dof_values)
                    self.env.UpdatePublishedBodies()
                    rospy.sleep(0.1)

    def getMesh(self, name):
        body = self.env.GetKinBody(name)
        if body == None:
            return None
        link = body.GetLinks()[0]
        col = link.GetCollisionData()
        return col.vertices, col.indices


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

    def generateNormalsSphere(self, angle):
        if angle <= 0:
            return None
        v_approach = []
	i = 0
        steps_alpha = int(math.pi/angle)
        if steps_alpha < 2:
            steps_alpha = 2
        for alpha in np.linspace(-90.0/180.0*math.pi, 90.0/180.0*math.pi, steps_alpha):
            max_steps_beta = (360.0/180.0*math.pi)/angle
            steps = int(math.cos(alpha)*max_steps_beta)
            if steps < 1:
                steps = 1
            beta_d = 360.0/180.0*math.pi/steps
            for beta in np.arange(0.0, 360.0/180.0*math.pi, beta_d):
                pt = PyKDL.Vector(math.cos(alpha)*math.cos(beta), math.cos(alpha)*math.sin(beta), math.sin(alpha))
                v_approach.append(pt)
#                self.pub_marker.publishSinglePointMarker(pt*0.1, i, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))
#                i += 1
                rospy.sleep(0.01)
        return v_approach

    def generateFramesForNormals(self, angle, normals):
        steps = int((360.0/180.0*math.pi)/angle)
        if steps < 2:
            steps = 2
        angle_d = 360.0/180.0*math.pi/steps
        frames = []
        for z in normals:
            if PyKDL.dot(z, PyKDL.Vector(0,0,1)) > 0.9:
                y = PyKDL.Vector(1,0,0)
            else:
                y = PyKDL.Vector(0,0,1)
            x = y * z
            y = z * x
            for angle in np.arange(0.0, 359.9/180.0*math.pi, angle_d):
                frames.append(PyKDL.Frame(PyKDL.Rotation(x,y,z)) * PyKDL.Frame(PyKDL.Rotation.RotZ(angle)))
                print angle/math.pi*180.0

        return frames

    def poseUpdaterThread(self, args, *args2):
        while not rospy.is_shutdown():
            for obj in self.objects:
                T_Br_M = self.getMarkerPose(obj[0])
                if T_Br_M != None:
                    T_Bo_M = obj[2]
                    T_Br_Bo = T_Br_M * T_Bo_M.Inverse()
                    self.openrave.updatePose(obj[1], T_Br_Bo)
            rospy.sleep(0.1)

    def pointInTriangle(self, A, B, C, P):
        # Compute vectors        
        v0 = [C[0] - A[0], C[1] - A[1]]
        v1 = [B[0] - A[0], B[1] - A[1]]
        v2 = [P[0] - A[0], P[1] - A[1]]

        # Compute dot products
        dot00 = v0[0]*v0[0] + v0[1]*v0[1]
        dot01 = v0[0]*v1[0] + v0[1]*v1[1]
        dot02 = v0[0]*v2[0] + v0[1]*v2[1]
        dot11 = v1[0]*v1[0] + v1[1]*v1[1]
        dot12 = v1[0]*v2[0] + v1[1]*v2[1]

        # Compute barycentric coordinates
        invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * invDenom
        v = (dot00 * dot12 - dot01 * dot02) * invDenom

        # Check if point is in triangle
        return (u >= 0) and (v >= 0) and (u + v < 1)

    def sampleMesh(self, vertices, indices, sample_dist, pt, radius, pt_list=None, radius2=None):#, min_dists=None):
        points = []
        for face in indices:
            A = vertices[face[0]]
            B = vertices[face[1]]
            C = vertices[face[2]]
            pt_a = PyKDL.Vector(A[0],A[1],A[2])
            pt_b = PyKDL.Vector(B[0],B[1],B[2])
            pt_c = PyKDL.Vector(C[0],C[1],C[2])
            v0 = pt_b - pt_a
            n0 = v0.Norm()
            steps0 = int(n0/sample_dist)
            if steps0 < 1:
                steps0 = 1
            step_len0 = n0/steps0

            v1 = pt_c - pt_a
            n1 = v1.Norm()
            angle = getAngle(v0,v1)
            h = n1*math.sin(angle)
            steps1 = int(h/sample_dist)
            if steps1 < 1:
                steps1 = 1
            step_len1 = h/steps1

            x0 = step_len0/2.0
            while x0 < n0:
                x1 = step_len1/2.0
                while x1 < h*(1.0-x0/n0):
                    point = pt_a + v0*(x0/n0) + v1*(x1/h)
                    if pt_list != None:
                        in_range = False
                        for s2 in pt_list:
                            if (point-s2).Norm() < radius2:
                                in_range = True
                                break
                        if in_range:
                            points.append(point)
                    elif (point-pt).Norm() < radius:
                        points.append(point)

                    x1 += step_len1
                x0 += step_len0

        if pt_list == None:
            return points

        min_dists = []
        min_dists_p_index = []
        for s in pt_list:
            min_dists.append(1000000.0)
            min_dists_p_index.append(None)

        i = 0
        for s in pt_list:
            p_index = 0
            for p in points:
                d = (s-p).Norm()
                if d < min_dists[i]:
                    min_dists[i] = d
                    min_dists_p_index[i] = p_index
                p_index += 1
            i += 1

        first_contact_index = None
        for i in range(0, len(pt_list)):
            if min_dists[i] < sample_dist*2.0:
                first_contact_index = i
                break

        init_pt = points[min_dists_p_index[first_contact_index]]
        points_ret = []
        list_to_check = []
        list_check_from = []
        for i in range(0, len(points)):
            if (init_pt-points[i]).Norm() > radius2:
                continue
            if i == min_dists_p_index[first_contact_index]:
                list_check_from.append(points[i])
            else:
                list_to_check.append(points[i])

#        print "points: %s"%(len(points))

        points_ret = []
        added_point = True
        iteration = 0
        while added_point:
            added_point = False
            list_close = []
            list_far = []
#            print "it: %s   points_ret: %s  list_check_from: %s  list_to_check: %s"%(iteration, len(points_ret), len(list_check_from), len(list_to_check))
            for p in list_to_check:
                added_p = False
                for check_from in list_check_from:
                    if (check_from-p).Norm() < sample_dist*2.0:
                        added_point = True
                        added_p = True
                        list_close.append(p)
                        break
                if not added_p:
                    list_far.append(p)

#            print "list_close: %s  list_far: %s"%(len(list_close), len(list_far))
            points_ret += list_check_from
            list_to_check = copy.deepcopy(list_far)
            list_check_from = copy.deepcopy(list_close)
            iteration += 1

        return points_ret

    def spin(self):
        # create the robot interface
        velma = Velma()

        self.openrave = OpenraveInstance(velma, PyKDL.Frame(PyKDL.Vector(0,0,0.1)))
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
#            points = self.sampleMesh(vertices, indices, 0.002, PyKDL.Vector(0.03,0,0.03), 0.04)
            points = self.sampleMesh(vertices, indices, 0.002, PyKDL.Vector(0.00,0,0.00), 0.04)
            print len(points)
            m_id = 0
            m_id = self.pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
            raw_input("Press Enter to continue...")

            rospy.sleep(5.0)

            pt_list = []
            for i in range(0, 20):
                pt_list.append(PyKDL.Vector((1.0*i/20.0)*0.1-0.05, 0, 0))
            points = self.sampleMesh(vertices, indices, 0.002, PyKDL.Vector(0.0,0,0.0), 0.04, pt_list, 0.01)
            print len(points)
            m_id = 0
            m_id = self.pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
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

#        rospy.sleep(5.0)
#        exit(0)
        grasps,indices = self.openrave.generateGrasps("object")

        min_cost = 10000.0
        min_i = 0
        for i in range(0, len(grasps)):
            T_Br_E = self.openrave.getGraspTransform(grasps[i], collisionfree=True)
#            self.openrave.showGrasp(grasps[i])
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

#        self.openrave.showGrasp(grasps[min_i])
        print "standoff: %s"%(self.openrave.getGraspStandoff(grasps[min_i]))

        raw_input("Press Enter to move the robot in " + str(duration) + " s...")
        if velma.checkStopCondition():
            exit(0)
        velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
        if velma.checkStopCondition(duration):
            exit(0)

        raw_input("Press Enter to close fingers...")

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
                points = self.sampleMesh(vertices, indices, 0.002, T_O_Br*c, 0.02, pt_list, 0.01)
                print len(points)
                m_id = self.pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.004, 0.004, 0.004), T=T_Br_O)
                rospy.sleep(2.0)
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
    pub_marker = MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()
    



