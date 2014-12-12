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
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *
from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from threading import RLock

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
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import velmautils
import operator
import random

class OpenraveInstance:

    def __init__(self, T_World_Br):
        self.robot = None
        self.rolling = False
        self.env = None
        self.T_World_Br = T_World_Br
        self.listener = tf.TransformListener();
        self.kinBodies = []
        self.visibility_surface_samples_dict = {}
        self.robot_rave_update_lock = RLock()

    def addRobotInterface(self, robot):
        self.robot = robot

    def KDLToOpenrave(self, T):
        ret = numpy.array([
        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
        [0, 0, 0, 1]])
        return ret

    def OpenraveToKDL(self, T):
        rot = PyKDL.Rotation(T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2])
        pos = PyKDL.Vector(T[0][3], T[1][3], T[2][3])
        return PyKDL.Frame(rot, pos)

    def addBox(self, name, x_size, y_size, z_size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromBoxes(numpy.array([[0,0,0,0.5*x_size,0.5*y_size,0.5*z_size]]),True)
        self.env.Add(body,True)

    def addCBeam(self, name, w, h, l, t):
#          h
#       _______
#    w  |
#       |______
#        
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromBoxes(numpy.array([
        [0.0, w/2.0 - t/2.0, 0.0, 0.5 * h, 0.5 * t, 0.5 * l],
        [-h/2.0 + t/2.0, 0.0, 0.0, 0.5 * t, 0.5 * (w - 2.0 * t), 0.5 * l],
        [0.0, -(w/2.0 - t/2.0), 0.0, 0.5 * h, 0.5 * t, 0.5 * l],
        ]),True)
        self.env.Add(body,True)

    def removeObject(self, name):
        with self.env:
            body = self.env.GetKinBody(name)
            self.env.Remove(body)

    def addSphere(self, name, size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromSpheres(numpy.array([[0,0,0,0.5*size]]),True)
        self.env.Add(body,True)
        return body

    def addCamera(self, name, fov_x, fov_y, dist):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        boxes = [
        [0,0,0,0.02,0.02,0.02],
        [dist*math.tan(fov_x/2.0),dist*math.tan(fov_y/2.0),dist,0.01,0.01,0.01],
        [-dist*math.tan(fov_x/2.0),dist*math.tan(fov_y/2.0),dist,0.01,0.01,0.01],
        [-dist*math.tan(fov_x/2.0),-dist*math.tan(fov_y/2.0),dist,0.01,0.01,0.01],
        [dist*math.tan(fov_x/2.0),-dist*math.tan(fov_y/2.0),dist,0.01,0.01,0.01],
        ]
        body.InitFromBoxes(numpy.array(boxes),True)
        self.env.Add(body,True)
#        self.env.CheckCollision(self.robot_rave,body)

    def updatePose(self, name, T_Br_Bo):
        with self.env:
            body = self.env.GetKinBody(name)
            if body != None:
                body.SetTransform(self.KDLToOpenrave(self.T_World_Br*T_Br_Bo))
            else:
#                print "openrave: could not find body: %s"%(name)
                pass
            self.env.UpdatePublishedBodies()

    def getPose(self, name):
        body = self.env.GetKinBody(name)
        if body != None:
            return self.T_World_Br.Inverse() * self.OpenraveToKDL(body.GetTransform())
        return None

    def getLinkPose(self, name, qt=None, qar=None, qal=None, qhr=None, qhl=None):
        if qt != None or qar != None or qal != None or qhr != None or qhl != None:
            self.robot_rave_update_lock.acquire()
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
                        T_World_L = self.OpenraveToKDL(link.GetTransform())
                        self.robot_rave_update_lock.release()
                        return self.T_World_Br.Inverse() * T_World_L
                    else:
                        self.robot_rave_update_lock.release()
                        return None
        else:
            link = self.robot_rave.GetLink(name)
            if link != None:
                return self.T_World_Br.Inverse() * self.OpenraveToKDL(link.GetTransform())
        return None

    def setCamera(self, T_Br_C):
        T_World_C = self.T_World_Br * T_Br_C
        self.env.GetViewer().SetCamera(self.KDLToOpenrave(T_World_C), 1.0)

    def main(self,env,options):
        try:
            self.wrist_collision_avoidance = velmautils.WristCollisionAvoidance("right", None, 5.0/180.0*math.pi)
            self.normals_sphere_30_deg = velmautils.generateNormalsSphere(60.0/180.0*math.pi)
            self.frames_60_deg = velmautils.generateFramesForNormals(60.0/180.0*math.pi, self.normals_sphere_30_deg)

            self.normals_sphere_5_deg = velmautils.generateNormalsSphere(5.0/180.0*math.pi, x_positive = True, y_positive = False)
            self.env = env
            self.robot_rave = env.ReadRobotXMLFile('robots/velma_col.robot.xml')

            # ODE does not support distance measure
            #self.env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts)
            self.env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)

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

            self.right_arm_dof_indices = []
            self.right_arm_wrist_dof_indices = []
            self.right_arm_not_wrist_dof_indices = []
            self.left_arm_dof_indices = []
            self.left_arm_wrist_dof_indices = []
            self.left_arm_not_wrist_dof_indices = []
            for j in self.robot_rave.GetJoints():
                if j.GetName().startswith("right_arm_"):
                    self.right_arm_dof_indices.append(j.GetJointIndex())
                    if j.GetName() == "right_arm_5_joint" or j.GetName() == "right_arm_6_joint":
                        self.right_arm_wrist_dof_indices.append(j.GetJointIndex())
                    else:
                        self.right_arm_not_wrist_dof_indices.append(j.GetJointIndex())
                if j.GetName().startswith("left_arm_"):
                    self.left_arm_dof_indices.append(j.GetJointIndex())
                    if j.GetName() == "left_arm_5_joint" or j.GetName() == "left_arm_6_joint":
                        self.left_arm_wrist_dof_indices.append(j.GetJointIndex())
                    else:
                        self.left_arm_not_wrist_dof_indices.append(j.GetJointIndex())
                print j

            self.right_palm_links_indices = []
            self.left_palm_links_indices = []
            for j in self.robot_rave.GetLinks():
                if j.GetName().startswith("right_Hand"):
                    self.right_palm_links_indices.append(j.GetIndex())
                if j.GetName().startswith("left_Hand"):
                    self.left_palm_links_indices.append(j.GetIndex())
                print j

            # apply soft limits
            q_soft_limit = 0.0#0.26*0.2
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
#            joint.SetLimits(lower, upper)
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

            # set velocity limits - it is not working
            vel_limits = []
            for i in range(0, self.robot_rave.GetDOF()):
                vel_limits.append(5.0/180.0*math.pi)
            self.robot_rave.SetDOFVelocityLimits(vel_limits)

            env.Add(self.robot_rave)

            joint = self.robot_rave.GetJoint("right_arm_5_joint")
            lower, upper = joint.GetLimits()
            print lower, upper

            self.robot_rave.SetActiveManipulator('right_arm')
            print "manipulator - arm indices: %s"%(self.robot_rave.GetActiveManipulator().GetArmIndices())
            print "manipulator - gripper indices: %s"%(self.robot_rave.GetActiveManipulator().GetGripperIndices())

            self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot_rave,iktype=IkParameterizationType.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()

#            self.ikmodel_translation3D = databases.inversekinematics.InverseKinematicsModel(self.robot_rave, iktype=IkParameterization.Type.Translation3D, freeindices=[self.robot_rave.GetJointIndex("right_arm_1_joint"), self.robot_rave.GetJointIndex("right_arm_4_joint"), self.robot_rave.GetJointIndex("right_arm_5_joint"), self.robot_rave.GetJointIndex("right_arm_6_joint")])
#            if not self.ikmodel_translation3D.load():
#                self.ikmodel_translation3D.autogenerate()

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
            
            env.GetViewer().SetCamera(self.KDLToOpenrave(cam_T), focalDistance)

#            self.robot_rave_update_lock.acquire()
#            self.robot_rave_update_lock.release()

            self.minimumgoalpaths = 1
            plannername = None
            self.basemanip = interfaces.BaseManipulation(self.robot_rave,plannername=plannername)
            self.basemanip.prob.SendCommand('SetMinimumGoalPaths %d'%self.minimumgoalpaths)

            # add torso
#            self.addBox("torso_box", 0.3, 1.0, 0.25)
#            self.obj_torso_box = self.env.GetKinBody("torso_box")
            self.obj_torso_box = None
            # add head
#            self.addSphere("head_sphere", 0.4)
#            self.obj_head_sphere = self.env.GetKinBody("head_sphere")
            self.obj_head_sphere = None

            while not rospy.is_shutdown():
                self.rolling = True
                if self.robot != None:
                    self.robot_rave_update_lock.acquire()
                    dof_values = self.robot.getAllDOFs()
                    self.robot_rave.SetDOFValues(dof_values)
                    self.robot_rave_update_lock.release()

                    # update head and torso
                    T_World_T2 = self.getLinkPose("torso_link2")
                    if self.obj_torso_box != None:
                        self.obj_torso_box.SetTransform(self.KDLToOpenrave(T_World_T2 * PyKDL.Frame(PyKDL.Vector(0, -0.40, 0))))
                    if self.obj_head_sphere != None:
                        self.obj_head_sphere.SetTransform(self.KDLToOpenrave(T_World_T2 * PyKDL.Frame(PyKDL.Vector(0.1, 0.57, 0))))
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

    def prepareGraspingModule(self, target_name, force_load=False):
        if not hasattr(self, 'gmodel') or self.gmodel == None:
            self.gmodel = {}

        if not target_name in self.gmodel.keys():
            target = self.env.GetKinBody(target_name)
            self.gmodel[target_name] = databases.grasping.GraspingModel(self.robot_rave,target)
            if force_load or not self.gmodel[target_name].load():
                print 'generating grasping model (one time computation)'
                self.gmodel[target_name].init(friction=0.9,avoidlinks=[])
                print 'grasping model initialised'
                print 'computing approach rays...'
                approachrays3 = self.gmodel[target_name].computeBoxApproachRays(delta=0.03,normalanglerange=0.0) #201, directiondelta=0.2)
#                print approachrays3.shape
#                print approachrays3[0]
#                exit(0)
#                approachrays3 = self.gmodel[target_name].computeBoxApproachRays(delta=0.03, normalanglerange=15.0/180.0*math.pi, directiondelta=14.0/180.0*math.pi)
#                approachrays3 = np.concatenate((approachrays, approachrays2), axis=0)
#                print approachrays3.shape
                print 'generating grasps...'
# possible arguments for generate:
# preshapes=None, standoffs=None, rolls=None, approachrays=None, graspingnoise=None, forceclosure=True, forceclosurethreshold=1.0000000000000001e-09, checkgraspfn=None, manipulatordirections=None, translationstepmult=None, finestep=None, friction=None, avoidlinks=None, plannername=None, boxdelta=None, spheredelta=None, normalanglerange=None
# http://openrave.org/docs/latest_stable/openravepy/databases.grasping/#openravepy.databases.grasping.GraspingModel.generatepcg
#                self.gmodel[target_name].generate(approachrays=approachrays3, forceclosure=False, standoffs=[0.025, 0.05, 0.075])
                self.gmodel[target_name].generate(approachrays=approachrays3, friction=0.9, forceclosure=True, standoffs=[0.04, 0.06, 0.07])
                self.gmodel[target_name].save()

    def getGraspsCount(self, target_name):
        self.prepareGraspingModule(target_name)
        return len(self.gmodel[target_name].grasps)

    def getGrasp(self, target_name, grasp_idx):
        self.prepareGraspingModule(target_name)
        return self.gmodel[target_name].grasps[grasp_idx]

    def generateGrasps(self, target_name, show=False, checkcollision=True, checkik=True, checkgrasper=True):
        self.prepareGraspingModule(target_name)

        validgrasps,validindices = self.gmodel[target_name].computeValidGrasps(checkcollision=checkcollision, checkik=checkik, checkgrasper=checkgrasper)
        print "all valid grasps: %s"%(len(validgrasps))

        print "done."
        return validindices

    def getGraspTransform(self, target_name, grasp, collisionfree=False):
        return self.T_World_Br.Inverse()*self.OpenraveToKDL( self.gmodel[target_name].getGlobalGraspTransform(grasp,collisionfree=collisionfree) )

    def showGrasp(self, target_name, grasp):
        self.robot_rave_update_lock.acquire()
        self.gmodel[target_name].showgrasp(grasp, collisionfree=False, useik=False)
        self.robot_rave_update_lock.release()

    def getGraspStandoff(self, target_name, grasp):
        return grasp[self.gmodel[target_name].graspindices.get('igraspstandoff')]

    def showTrajectory(self, time, qt_list=None, qar_list=None, qal_list=None, qhr_list=None, qhl_list=None):
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
            return None
        time_d = time / length
        report = CollisionReport()
        first_collision = None
        self.robot_rave_update_lock.acquire()
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
                    dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                    self.robot_rave.SetDOFValues(dof_values)
                    if time_d > 0.0:
                        self.env.UpdatePublishedBodies()
                    check = self.env.CheckCollision(self.robot_rave, report)
                    if first_collision == None and report.numCols > 0:
                        first_collision = i
                        print "first collision at step %s"%(i)
                    if time_d > 0.0:
                        rospy.sleep(time_d)
        self.robot_rave_update_lock.release()
        return first_collision

    def getMesh(self, name):
        body = self.env.GetKinBody(name)
        if body == None:
            return None
        link = body.GetLinks()[0]
        col = link.GetCollisionData()
        return col.vertices, col.indices

    def getFinalConfig(self, target_name, grasp, show=False):
        hand_config = None
        contacts = None
        self.robot_rave_update_lock.acquire()
        with self.robot_rave.CreateRobotStateSaver():
            with self.gmodel[target_name].GripperVisibility(self.robot_rave.GetActiveManipulator()):
#            with self.env:
                try:
#                    self.gmodel[target_name].setPreshape(grasp)
#                    Tgrasp = self.gmodel[target_name].getGlobalGraspTransform(grasp,collisionfree=False)
#                    Tdelta = np.dot(Tgrasp,np.linalg.inv(self.robot_rave.GetActiveManipulator().GetEndEffectorTransform()))
#                    for link in self.robot_rave.GetActiveManipulator().GetChildLinks():
#                        link.SetTransform(np.dot(Tdelta,link.GetTransform()))

                    contacts,finalconfig,mindist,volume = self.gmodel[target_name].runGraspFromTrans(grasp)
#                    contacts,finalconfig,mindist,volume = self.gmodel[target_name].runGrasp(grasp)
                    hand_config = [
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerOneKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerTwoKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerThreeKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerOneKnuckleOneJoint")],
                    ]

                    ind = self.robot_rave.GetActiveManipulator().GetGripperIndices()
                    hand_config2 = [
                    finalconfig[0][ind[0]],
                    finalconfig[0][ind[1]],
                    finalconfig[0][ind[2]],
                    finalconfig[0][ind[3]],
                    ]

                    if show:
                        self.robot_rave.SetTransform(np.dot(self.gmodel[target_name].getGlobalGraspTransform(grasp),np.dot(np.linalg.inv(self.robot_rave.GetActiveManipulator().GetEndEffectorTransform()),self.robot_rave.GetTransform())))

#                        self.robot_rave.SetDOFValues([hand_config[3], hand_config[0], hand_config[1], hand_config[2]],self.robot_rave.GetActiveManipulator().GetGripperIndices())
                        self.robot_rave.SetDOFValues([hand_config2[0], hand_config2[1], hand_config2[2], hand_config2[3]],self.robot_rave.GetActiveManipulator().GetGripperIndices())
#                        self.robot_rave.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')],self.manip.GetGripperIndices())

                        self.env.UpdatePublishedBodies()
                        raw_input('press any key to continue: ')

#Tgrasp = self.getGlobalGraspTransform(grasp,collisionfree=collisionfree)
#Tdelta = dot(Tgrasp,linalg.inv(self.manip.GetEndEffectorTransform())) for link in self.manip.GetChildLinks(): link.SetTransform(dot(Tdelta,link.GetTransform())) self.env.UpdatePublishedBodies() # wait while environment is locked? if delay is None: raw_input('press any key to continue: ') elif delay > 0: time.sleep(delay)





                except planning_error,e:
                    print "getFinalConfig: planning error:"
                    print e
        self.robot_rave_update_lock.release()
        if contacts == None:
            contacts_ret = None
        else:
            contacts_ret = []
            for c in contacts:
                contacts_ret.append(self.T_World_Br.Inverse() * PyKDL.Vector(c[0], c[1], c[2]))
        return hand_config, contacts_ret

    def grab(self, name):
        body = self.env.GetKinBody(name)
        self.robot_rave_update_lock.acquire()
        with self.env:
            self.robot_rave.Grab( self.env.GetKinBody(name))
        self.robot_rave_update_lock.release()

    def release(self, name):
        self.robot_rave_update_lock.acquire()
        with self.env:
            self.robot_rave.ReleaseAllGrabbed()
        self.robot_rave_update_lock.release()

    def getVisibility(self, name, T_Br_C, qt=None, qar=None, qal=None, qhr=None, qhl=None, pub_marker=None, fov_x=None, fov_y=None, min_dist=0.01):
        # remove the head sphere from environment
        self.env.Remove(self.obj_head_sphere)
        if name in self.visibility_surface_samples_dict:
            points = self.visibility_surface_samples_dict[name]
        else:
            vertices, indices = self.getMesh(name)
            points = velmautils.sampleMesh(vertices, indices, 0.025, [PyKDL.Vector()], 1.0)
            print "points for visibility test: %s"%(len(points))
            self.visibility_surface_samples_dict[name] = points
        T_World_C = self.T_World_Br * T_Br_C
        T_C_World = T_World_C.Inverse()
        R_C_World = PyKDL.Frame(T_C_World.M)
        cam_pt_in_World = T_World_C * PyKDL.Vector()
        if fov_x != None and fov_y != None:
            tg_fov_x = math.tan(fov_x/2.0)
            tg_fov_y = math.tan(fov_y/2.0)
        m_id = 0
        if qt != None or qar != None or qal != None or qhr != None or qhl != None:
            self.robot_rave_update_lock.acquire()
            with self.env:
                with self.robot_rave.CreateRobotStateSaver():
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
                    body = self.env.GetKinBody(name)
                    T_World_O = self.OpenraveToKDL(body.GetTransform())
                    hits = 0
                    all_hits = 0
                    for p in points:
                        p_in_World = T_World_O * p
                        d = p_in_World - cam_pt_in_World
                        d_len = d.Norm()
                        # elongate the ray by 1 cm
                        d = d * (d_len + 0.01)/d_len
                        check_collision = True
                        if fov_x != None and fov_y != None:
                            # check the ray in camera frame
                            d_cam = R_C_World * d
                            if d_cam.z() < min_dist or math.fabs(d_cam.x()/d_cam.z()) > tg_fov_x or math.fabs(d_cam.y()/d_cam.z()) > tg_fov_y:
                                check_collision = False
                        if check_collision:
                            report = CollisionReport()
                            ret = self.env.CheckCollision(Ray([cam_pt_in_World.x(),cam_pt_in_World.y(),cam_pt_in_World.z()],[d.x(),d.y(),d.z()]), report)
                            if pub_marker != None:
                                m_id = pub_marker.publishVectorMarker(cam_pt_in_World, cam_pt_in_World+d, m_id, 1, 1, 1, frame='world', namespace='default', scale=0.001)
                            if report.numCols > 0:
                                all_hits += 1
                                parent = report.plink1.GetParent()
                                if parent.GetName() == name:
                                    hits += 1
#                    print "all_rays: %s   all_hits: %s   hits: %s"%(len(points), all_hits, hits)
            self.robot_rave_update_lock.release()
        else:
            body = self.env.GetKinBody(name)

            T_World_O = self.OpenraveToKDL(body.GetTransform())
            hits = 0
            all_hits = 0
            for p in points:
                p_in_World = T_World_O * p
                d = p_in_World - cam_pt_in_World
                d_len = d.Norm()
                d = d * (d_len + 0.01)/d_len
                check_collision = True
                if fov_x != None and fov_y != None:
                    # check the ray in camera frame
                    d_cam = R_C_World * d
                    if d_cam.z() < min_dist or math.fabs(d_cam.x()/d_cam.z()) > tg_fov_x or math.fabs(d_cam.y()/d_cam.z()) > tg_fov_y:
                        check_collision = False
                if check_collision:
                    report = CollisionReport()
                    ret = self.env.CheckCollision(Ray([cam_pt_in_World.x(),cam_pt_in_World.y(),cam_pt_in_World.z()],[d.x(),d.y(),d.z()]), report)
                    if pub_marker != None:
                        m_id = pub_marker.publishVectorMarker(cam_pt_in_World, cam_pt_in_World+d, m_id, 1, 1, 1, frame='world', namespace='default', scale=0.001)
                    if report.numCols > 0:
                        all_hits += 1
                        parent = report.plink1.GetParent()
                        if parent.GetName() == name:
                            hits += 1

#            print "all_rays: %s   all_hits: %s   hits: %s"%(len(points), all_hits, hits)
        # add the head sphere from environment
        self.env.Add(self.obj_head_sphere, True)

        return float(hits)/float(len(points))

    def findIkSolution(self, T_Br_E):
        return self.ikmodel.manip.FindIKSolution(self.KDLToOpenrave(self.T_World_Br * T_Br_E), IkFilterOptions.CheckEnvCollisions)

    def findIkSolutions(self, T_Br_E):
        return self.ikmodel.manip.FindIKSolutions(self.KDLToOpenrave(self.T_World_Br * T_Br_E), IkFilterOptions.CheckEnvCollisions)

    def findIkSolutionsTranslation3D(self, T_Br_E):
        return self.ikmodel_translation3D.manip.FindIKSolutions(self.KDLToOpenrave(self.T_World_Br * T_Br_E), IkFilterOptions.CheckEnvCollisions)

    def getBestFinalConfigs(self, T_Br_E):
        q_list = self.findIkSolutions(T_Br_E)
        print "ik solutions: %s"%(len(q_list))
        q_score = []
        lower_lim, upper_lim = self.robot_rave.GetDOFLimits(self.right_arm_dof_indices)
        print lower_lim
        print upper_lim
        for q in q_list:
            q_score.append([1000000.0, q])
            # punish for singularities in end configuration
            if abs(q[1]) < 30.0/180.0*math.pi:
                continue
            if abs(q[3]) < 30.0/180.0*math.pi:
                continue
            if abs(q[5]) < 30.0/180.0*math.pi:
                continue

            score = 0.0

            if abs(q[1]) < 40.0/180.0*math.pi:
                score += 40.0/180.0*math.pi - abs(q[1])
            if abs(q[3]) < 40.0/180.0*math.pi:
                score += 40.0/180.0*math.pi - abs(q[3])
            if abs(q[5]) < 40.0/180.0*math.pi:
                score += 40.0/180.0*math.pi - abs(q[5])

            score *= 10.0

            for i in range(0, 7):
                score += (self.robot.qar[i]-q[i])*(self.robot.qar[i]-q[i])
                if abs(q[i]-lower_lim[i]) < 40.0/180.0*math.pi:
                    score += 40.0/180.0*math.pi - abs(q[i]-lower_lim[i])
                if abs(q[i]-upper_lim[i]) < 40.0/180.0*math.pi:
                    score += 40.0/180.0*math.pi - abs(q[i]-upper_lim[i])
            q_score[-1][0] = score

        q_sorted = sorted(q_score, key=operator.itemgetter(0))
        return q_sorted

    def findFreeSpaceSphere(self, radius, dist_from_shoulder):
        self.robot_rave_update_lock.acquire()

        # first, find the free space in the best area for manipulation
        with self.robot_rave.CreateRobotStateSaver():
                    self.robot_rave.GetController().Reset(0)
                    dof_values = self.robot_rave.GetDOFValues()
                    qt = dof_values[0:2]
                    qal = dof_values[2:9]
                    qhl = dof_values[9:13]
                    qar = [0.0, -90.0/180*math.pi, 0.0, 0.0, 0.0, 0.0, 0.0]
                    qhr = dof_values[20:24]
                    dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                    self.robot_rave.SetDOFValues(dof_values)

                    with self.env:
                        wrist_sphere = self.addSphere("wrist_sphere", 2.0 * radius)
                        T_Br_T2 = self.getLinkPose("torso_link2")
                        T_Br_L2r = self.getLinkPose("right_arm_2_link")
                        Pc_T2 = T_Br_T2.Inverse() * T_Br_L2r * PyKDL.Vector()
                        it = 0
                        while True:
                            v = self.normals_sphere_5_deg[random.randint(0, len(self.normals_sphere_5_deg)-1)]
                            P_Br = T_Br_T2 * (Pc_T2 + v * dist_from_shoulder)
                            wrist_sphere.SetTransform(self.KDLToOpenrave(self.T_World_Br*PyKDL.Frame(PyKDL.Vector(P_Br))))
                            report = CollisionReport()
                            self.env.CheckCollision(wrist_sphere, report)

                            if report.numCols == 0:
                                break
                            it += 1
                            if it > 200:
                                P_Br = None
                                break
                        self.env.Remove(wrist_sphere)

        self.robot_rave_update_lock.release()

        return P_Br

    def planMoveToFreeSpace(self):
        self.robot_rave_update_lock.acquire()
        P_Br = self.findFreeSpaceSphere(0.35, 0.5)
        if P_Br == None:
            print "planMoveToFreeSpace: could not find free area"
            return None

        init_q = self.robot_rave.GetDOFValues(self.right_arm_dof_indices)
        init_sect = self.wrist_collision_avoidance.getQ5Q6SpaceSectors(init_q[5], init_q[6])
        print "init_sect: %s"%(init_sect)
        if len(init_sect) == 0:
            init_sect = [ self.wrist_collision_avoidance.getClosestQ5Q6SpaceSector(init_q[5], init_q[6]) ]
            print "planMoveToFreeSpace: wrong starting position in q5: %s and q6: %s"%(init_q[5], init_q[6])

        with self.robot_rave:
                    try:
                        init_q = self.robot_rave.GetDOFValues(self.right_arm_dof_indices)
                        self.robot_rave.SetActiveDOFs(self.right_arm_dof_indices)
                        # set the intermediate point
                        T_Br_E_int = PyKDL.Frame(P_Br)
                        T_Br_E_current = self.getLinkPose("right_HandPalmLink")

                        print "frames: %s"%(len(self.frames_60_deg))
                        q_list = []
                        for fr in self.frames_60_deg:
                            diff1 = PyKDL.diff(fr, T_Br_E_current)
                            if diff1.rot.Norm() > 90.0/180.0*math.pi:
                                continue
                            sol = list(self.findIkSolutions(T_Br_E_int * fr))
                            print len(sol)
                            q_list += sol
                        print "found solutions: %s"%(len(q_list))

                        min_score = 1000000.0
                        min_q_sol = None
                        # get the closest solution to the current and the end configuration
                        for q_sol in q_list:
                            end_sect = self.wrist_collision_avoidance.getQ5Q6SpaceSectors(q_sol[5], q_sol[6])
                            same_sector = False
                            for s in init_sect:
                                if s in end_sect:
                                    same_sector = True
                                    break
                            if not same_sector:
                                continue
                            score = 0.0
                            for q_idx in range(0, 5):
                                score += (q_sol[q_idx]-init_q[q_idx])*(q_sol[q_idx]-init_q[q_idx])
                            if score < min_score:
                                min_score = score
                                min_q_sol = q_sol
                    except planning_error,e:
                        pass
        self.robot_rave_update_lock.release()
        return min_q_sol

    def planMoveThroughGoals(self, goals, goal0_wrist_only=False):
        traj = None
        self.robot_rave_update_lock.acquire()
        try:
            with self.robot_rave:
                self.robot_rave.SetActiveDOFs(self.right_arm_dof_indices)
                if goal0_wrist_only:
                    self.robot_rave.GetLink("right_arm_5_link").Enable(False)
                    traj = self.basemanip.MoveActiveJoints(goal=goals[0],execute=False,outputtrajobj=True)
                    self.robot_rave.GetLink("right_arm_5_link").Enable(True)
                else:
                    traj = self.basemanip.MoveActiveJoints(goal=goals[0],execute=False,outputtrajobj=True)
                for idx in range(1, len(goals)):
                    with self.robot_rave.CreateRobotStateSaver():
                        self.robot_rave.SetActiveDOFs(self.right_arm_dof_indices)
                        self.robot_rave.GetController().Reset(0)
                        dof_values = self.robot_rave.GetDOFValues()
                        qt = dof_values[0:2]
                        qal = dof_values[2:9]
                        qhl = dof_values[9:13]
                        qar = goals[idx-1]
                        qhr = dof_values[20:24]
                        dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                        print dof_values
                        self.robot_rave.SetDOFValues(dof_values)
                        t = self.basemanip.MoveActiveJoints(goal=goals[idx],execute=False,outputtrajobj=True)
                    print "trajectory len: %s"%(t.GetNumWaypoints())
                    if t.GetNumWaypoints() > 1:
                        traj.Insert(traj.GetNumWaypoints(), t.GetWaypoints(0, t.GetNumWaypoints()), False)
        except planning_error,e:
            print "planMoveThroughGoals: planning error"
        self.robot_rave_update_lock.release()
        return traj

    def planMoveForRightArm(self, T_Br_E, q_dest, maxiter=500, verbose_print=False):
        self.robot_rave_update_lock.acquire()

        if T_Br_E != None and q_dest == None:
            q_list = self.getBestFinalConfigs(T_Br_E)
            if len(q_list) < 30:
                self.robot_rave_update_lock.release()
                print "planMoveForRightArm: strange pose - a few ik solutions"
                return None
        elif T_Br_E == None and q_dest != None:
            q_list = [[0.0, q_dest]]
        else:
            self.robot_rave_update_lock.release()
            print "planMoveForRightArm: wrong arguments: %s %s"%(T_Br_E, q_dest)
            return None

        init_q = self.robot_rave.GetDOFValues(self.right_arm_dof_indices)
        init_sect = self.wrist_collision_avoidance.getQ5Q6SpaceSectors(init_q[5], init_q[6])
        print "init_sect: %s"%(init_sect)
        if len(init_sect) == 0:
#            init_sect = [ self.wrist_collision_avoidance.getClosestQ5Q6SpaceSector(init_q[5], init_q[6]) ]
            print "planMoveForRightArm: wrong starting position in q5: %s and q6: %s"%(init_q[5], init_q[6])
        for q_s in q_list:

            traj = None
            q = q_s[1]
            end_sect = self.wrist_collision_avoidance.getQ5Q6SpaceSectors(q[5], q[6])
            print "end_sect: %s"%(end_sect)
            if len(end_sect) == 0:
                continue

            score = q_s[0]
            print score
            if score > 10000.0:
                break

            goal0_wrist_only = False
            goals = []
            # the starting position is outside the safe q5-q6 area
            if len(init_sect) == 0:
                closest_sect = self.wrist_collision_avoidance.getClosestQ5Q6SpaceSector(q[5], q[6])
                q5_d, q6_d = self.wrist_collision_avoidance.forceMoveQ5Q6ToSector(q[5], q[6], closest_sect)
                goals.append(np.array([init_q[0], init_q[1], init_q[2], init_q[3], init_q[4], q5_d, q6_d]))
                init_sect = [ closest_sect ]
                goal0_wrist_only = True
            same_sector = False
            for s in init_sect:
                if s in end_sect:
                    same_sector = True
                    break
            # q5 and q6 are in the same sector - there should be no problem in path execution
            if same_sector:
                goals.append(q)
            else:
                # q5 and q6 are in different sectors - we have to move the wrist to the safe area first
                q5q6_traj = self.wrist_collision_avoidance.getQ5Q6Traj(init_q[5], init_q[6], q[5], q[6])

                P_Br = self.findFreeSpaceSphere(0.35, 0.5)
                if P_Br == None:
                    print "planMoveForRightArm: could not find free area"

                with self.robot_rave:
                    try:
#                        self.robot_rave.SetActiveDOFs(self.right_arm_not_wrist_dof_indices)
                        self.robot_rave.SetActiveDOFs(self.right_arm_dof_indices)
                        # set the intermediate point
                        T_Br_E_int = PyKDL.Frame(P_Br)
                        T_Br_E_current = self.getLinkPose("right_HandPalmLink")

                        print "frames: %s"%(len(self.frames_60_deg))
                        q_list2 = []
                        for fr in self.frames_60_deg:
                            diff1 = PyKDL.diff(fr, T_Br_E_current)
                            if diff1.rot.Norm() > 90.0/180.0*math.pi:
                                continue
                            sol = list(self.findIkSolutions(T_Br_E_int * fr))
                            print len(sol)
                            q_list2 += sol
                        print "found solutions: %s"%(len(q_list2))

                        min_score = 1000000.0
                        min_q_sol = None
                        # get the closest solution to the current and the end configuration
                        for q_sol in q_list2:
                            end_sect = self.wrist_collision_avoidance.getQ5Q6SpaceSectors(q_sol[5], q_sol[6])
                            same_sector = False
                            for s in init_sect:
                                if s in end_sect:
                                    same_sector = True
                                    break
                            if not same_sector:
                                continue
                            score = 0.0
                            for q_idx in range(0, 5):
                                score += (q_sol[q_idx]-init_q[q_idx])*(q_sol[q_idx]-init_q[q_idx])
                                score += (q_sol[q_idx]-q[q_idx])*(q_sol[q_idx]-q[q_idx])
                            if score < min_score:
                                min_score = score
                                min_q_sol = q_sol

                        goals = []
                        goals.append(min_q_sol)

                        for q5q6 in q5q6_traj:
                            goals.append(np.array([min_q_sol[0], min_q_sol[1], min_q_sol[2], min_q_sol[3], min_q_sol[4], q5q6[0], q5q6[1]]))
                        goals.append(q)
                    except planning_error,e:
                        pass

#            print "goals count: %s"%(len(goals))
#            print "goals:"
#            print goals
            traj = self.planMoveThroughGoals(goals, goal0_wrist_only=goal0_wrist_only)
            if traj == None:
                print "error: planMoveThroughGoals"
                continue

            # verify the trajectory
            conf = traj.GetConfigurationSpecification()
            q_traj = []
            steps2 = max(2, int(traj.GetDuration()*200.0))
            q5q6_collision = False
            first_q5q6_collision = None
            for t in np.linspace(0.0, traj.GetDuration(), steps2):
                q = conf.ExtractJointValues(traj.Sample(t), self.robot_rave, self.right_arm_dof_indices)
                q_traj.append(list(q))
                if len(self.wrist_collision_avoidance.getQ5Q6SpaceSectors(q[5], q[6])) == 0:
                    if first_q5q6_collision == None:
                        first_q5q6_collision = t
                    q5q6_collision = True

            if q5q6_collision:
                # TODO: better handle the q5-q6 collision
                print "q5q6 collision"
#                continue
            break

        self.robot_rave_update_lock.release()

        if traj == None:
            print "planMoveForRightArm: planning error"
            return None

        if q5q6_collision:
            print "planMoveForRightArm: q5-q6 collision: %s / %s"%(first_q5q6_collision, traj.GetDuration())
#            return None

        if verbose_print:
            print "all groups:"
            for gr in conf.GetGroups():
                print gr.name

        def printGroup(gr):
            print "offset: %s   dof: %s   name: %s   interpolation: %s"%(gr.offset, gr.dof, gr.name, gr.interpolation)

        try:
            gr_tim = conf.GetGroupFromName("deltatime")
            tim = []
            if verbose_print:
                print "gr_tim:"
                printGroup(gr_pos)
        except openrave_exception:
            gr_tim = None
            tim = None
            if verbose_print:
                print "gr_tim == None"
        try:
            gr_pos = conf.GetGroupFromName("joint_values")
            pos = []
            if verbose_print:
                print "gr_pos:"
                printGroup(gr_pos)
        except openrave_exception:
            gr_pos = None
            pos = None
            if verbose_print:
                print "gr_pos == None"
        try:
            gr_vel = conf.GetGroupFromName("joint_velocities")
            vel = []
            if verbose_print:
                print "gr_vel:"
                printGroup(gr_vel)
        except openrave_exception:
            gr_vel = None
            vel = None
            if verbose_print:
                print "gr_vel == None"
        try:
            gr_acc = conf.GetGroupFromName("joint_accelerations")
            acc = []
            if verbose_print:
                print "gr_acc:"
                printGroup(gr_acc)
        except openrave_exception:
            gr_acc = None
            acc = None
            if verbose_print:
                print "gr_acc == None"

        if verbose_print:
            print "waypoints: %s"%(traj.GetNumWaypoints())

        for idx in range(0, traj.GetNumWaypoints()):
            w = traj.GetWaypoint(idx)
            if pos != None:
                pos.append( [w[gr_pos.offset], w[gr_pos.offset + 1], w[gr_pos.offset + 2], w[gr_pos.offset + 3], w[gr_pos.offset + 4], w[gr_pos.offset + 5], w[gr_pos.offset + 6]] )
            if vel != None:
               vel.append( [w[gr_vel.offset], w[gr_vel.offset + 1], w[gr_vel.offset + 2], w[gr_vel.offset + 3], w[gr_vel.offset + 4], w[gr_vel.offset + 5], w[gr_vel.offset + 6]] )
            if acc != None:
               acc.append( [w[gr_acc.offset], w[gr_acc.offset + 1], w[gr_acc.offset + 2], w[gr_acc.offset + 3], w[gr_acc.offset + 4], w[gr_acc.offset + 5], w[gr_acc.offset + 6]] )
            if tim != None:
               tim.append( w[gr_tim.offset] )

        if verbose_print:
            print "pos"
            print pos
            print "tim"
            print tim
            if tim != None:
                print "tim sum: %s"%(math.fsum(tim))
            print "duration: %s"%(traj.GetDuration())
        return pos, vel, acc, tim, q_traj, q5q6_collision

    def printCollisions(self):
        report = CollisionReport()
        if self.env.CheckCollision(self.robot_rave, report):
            print 'robot in collision:'
            if report.plink1 != None:
                print report.plink1
            if report.plink2 != None:
                print report.plink2
            if report.numCols != None:
                print report.numCols
        else:
            print "no collisions"

    def checkGripperCollision(self, target_name, grasp_idx):
        collision = False
        grasp = self.gmodel[target_name].grasps[grasp_idx]
        self.robot_rave_update_lock.acquire()
        with self.robot_rave.CreateRobotStateSaver():
            with self.gmodel[target_name].GripperVisibility(self.gmodel[target_name].manip):
                with self.env:
                    self.gmodel[target_name].setPreshape(grasp)
                    Tgrasp = self.gmodel[target_name].getGlobalGraspTransform(grasp,collisionfree=False)
                    Tdelta = np.dot(Tgrasp,np.linalg.inv(self.gmodel[target_name].manip.GetEndEffectorTransform()))
                    for link in self.gmodel[target_name].manip.GetChildLinks():
                        link.SetTransform(np.dot(Tdelta,link.GetTransform()))
                    report = CollisionReport()
                    if self.env.CheckCollision(self.robot_rave, report):
                        collision = True
        self.robot_rave_update_lock.release()
        return collision


