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
import numpy as np
import copy
import thread
from scipy import optimize
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

class OpenraveInstance:

    def __init__(self, T_World_Br):
        self.robot = None
        self.rolling = False
        self.env = None
        self.T_World_Br = T_World_Br
        self.listener = tf.TransformListener();
        self.kinBodies = []

    def addRobotInterface(self, robot):
        self.robot = robot

    def KDLToOrocos(self, T):
        ret = numpy.array([
        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
        [0, 0, 0, 1]])
        return ret

    def orocosToKDL(self, T):
        rot = PyKDL.Rotation(T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2])
        pos = PyKDL.Vector(T[0][3], T[1][3], T[2][3])
        return PyKDL.Frame(rot, pos)

    def addBox(self, name, x_size, y_size, z_size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromBoxes(numpy.array([[0,0,0,0.5*x_size,0.5*y_size,0.5*z_size]]),True)
        self.env.Add(body,True)

    def updatePose(self, name, T_Br_Bo):
        with self.env:
            body = self.env.GetKinBody(name)
            if body != None:
                body.SetTransform(self.KDLToOrocos(self.T_World_Br*T_Br_Bo))
            else:
                print "openrave: could not find body: %s"%(name)
                self.env.UpdatePublishedBodies()

    def getPose(self, name):
        body = self.env.GetKinBody(name)
        if body != None:
            return self.T_World_Br.Inverse() * self.orocosToKDL(body.GetTransform())
        return None

    def getLinkPose(self, name, qt=None, qar=None, qal=None, qhr=None, qhl=None):
        if qt != None or qar != None or qal != None or qhr != None or qhl != None:
            with self.robot_rave.CreateRobotStateSaver():
                with self.env:
                    self.robot_rave.GetController().Reset(0)
                    dof_values = self.robot_rave.GetDOFValues()
                    if qt == None:
                        qt = dof_values[0:2]
                    if qal == None:
                        qal = dof_values[2:9]
                    if qhl == None:
                        qhl = dof_values[9:13]
                    if qar == None:
                        qar = dof_values[13:20]
                    if qhr == None:
                        qhr = dof_values[20:24]
                    dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                    self.robot_rave.SetDOFValues(dof_values)
                    self.env.UpdatePublishedBodies()
                    link = self.robot_rave.GetLink(name)
                    if link != None:
                        return self.T_World_Br.Inverse() * self.orocosToKDL(link.GetTransform())
        else:
            link = self.robot_rave.GetLink(name)
            if link != None:
                return self.T_World_Br.Inverse() * self.orocosToKDL(link.GetTransform())
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
            q_soft_limit = 0.26*0.5
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
#            joint.SetLimits(lower, upper)
            
            # apply limits for wrists
            joint = self.robot_rave.GetJoint("left_arm_5_joint")
            lower, upper = joint.GetLimits()
            lower[0] = 20.0/180.0*math.pi
#            joint.SetLimits(lower, upper)
            joint = self.robot_rave.GetJoint("right_arm_5_joint")
            lower, upper = joint.GetLimits()
            print lower, upper
            upper[0] = -20.0/180.0*math.pi
#            joint.SetLimits(lower, upper)

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
            
            env.GetViewer().SetCamera(self.KDLToOrocos(cam_T), focalDistance)

            while not rospy.is_shutdown():
                self.rolling = True
                if self.robot != None:
#                    q_lf = [self.robot.q_lf[0], self.robot.q_lf[1], self.robot.q_lf[4], self.robot.q_lf[6]]
#                    q_rf = [self.robot.q_rf[0], self.robot.q_rf[1], self.robot.q_rf[4], self.robot.q_rf[6]]
#                    dof_values = self.robot.q_t + self.robot.q_l + q_lf + self.robot.q_r + q_rf

                    dof_values = self.robot.qt + self.robot.qal + self.robot.qhl + self.robot.qar + self.robot.qhr

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

        print "done."
        return validgrasps,validindices

    def getGraspTransform(self, grasp, collisionfree=False):
        return self.T_World_Br.Inverse()*self.orocosToKDL( self.gmodel.getGlobalGraspTransform(grasp,collisionfree=collisionfree) )

    def showGrasp(self, grasp):
        self.gmodel.showgrasp(grasp, collisionfree=True, useik=True)

    def getGraspStandoff(self, grasp):
        return grasp[self.gmodel.graspindices.get('igraspstandoff')]

    def showTrajectory(self, time_d, qt_list=None, qar_list=None, qal_list=None, qhr_list=None, qhl_list=None):
        length = 0
        if qt_list != None:
            length = len(qt_list)
        elif qar_list != None:
            length = len(qar_list)
        elif qal_list != None:
            length = len(qal_list)
        elif qhr_list != None:
            length = len(qhr_list)
        elif qhl_list != None:
            length = len(qhl_list)
        if length < 1:
            return
        with self.robot_rave.CreateRobotStateSaver():
            with self.env:
                for i in range(0, length):
                    self.robot_rave.GetController().Reset(0)
                    dof_values = self.robot_rave.GetDOFValues()
                    if qt_list == None:
                        qt = dof_values[0:2]
                    else:
                        qt = qt_list[i]
                    if qal_list == None:
                        qal = dof_values[2:9]
                    else:
                        qal = qal_list[i]
                    if qhl_list == None:
                        qhl = dof_values[9:13]
                    else:
                        qhl = qhl_list[i]
                    if qar_list == None:
                        qar = dof_values[13:20]
                    else:
                        qar = qar_list[i]
                    if qhr_list == None:
                        qhr = dof_values[20:24]
                    else:
                        qhr = qhr_list[i]
                    print "qt: %s"%(qt)
                    print "qal: %s"%(qal)
                    print "qhl: %s"%(qhl)
                    print "qar: %s"%(qar)
                    print "qhr: %s"%(qhr)
                    dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                    self.robot_rave.SetDOFValues(dof_values)
                    self.env.UpdatePublishedBodies()
                    rospy.sleep(time_d)

    def getMesh(self, name):
        body = self.env.GetKinBody(name)
        if body == None:
            return None
        link = body.GetLinks()[0]
        col = link.GetCollisionData()
        return col.vertices, col.indices

    def getFinalConfig(self, grasp):
        with self.robot_rave.CreateRobotStateSaver():
            with self.env:
                contacts,finalconfig,mindist,volume = self.gmodel.runGraspFromTrans(grasp)
                hand_config = [
                finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerOneKnuckleTwoJoint")],
                finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerTwoKnuckleTwoJoint")],
                finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerThreeKnuckleTwoJoint")],
                finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerOneKnuckleOneJoint")],
                ]
        return hand_config

