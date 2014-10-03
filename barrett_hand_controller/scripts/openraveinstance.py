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
import velmautils

class OpenraveInstance:

    def __init__(self, T_World_Br):
        self.robot = None
        self.rolling = False
        self.env = None
        self.T_World_Br = T_World_Br
        self.listener = tf.TransformListener();
        self.kinBodies = []
        self.visibility_surface_samples_dict = {}
        self.robot_rave_update_lock = Lock()

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

    def removeObject(self, name):
        with self.env:
            body = self.env.GetKinBody(name)
            self.env.Remove(body)

    def addSphere(self, name, size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromSpheres(numpy.array([[0,0,0,0.5*size]]),True)
        self.env.Add(body,True)

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
            self.left_arm_dof_indices = []
            for j in self.robot_rave.GetJoints():
                if j.GetName().startswith("right_arm_"):
                    self.right_arm_dof_indices.append(j.GetJointIndex())
                if j.GetName().startswith("left_arm_"):
                    self.left_arm_dof_indices.append(j.GetJointIndex())
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
            q_soft_limit = 0.26*0.2
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

            env.Add(self.robot_rave)

            joint = self.robot_rave.GetJoint("right_arm_5_joint")
            lower, upper = joint.GetLimits()
            print lower, upper

            self.robot_rave.SetActiveManipulator('right_arm')

            self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot_rave,iktype=IkParameterizationType.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()

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
            self.addBox("torso_box", 0.3, 1.0, 0.25)
            self.obj_torso_box = self.env.GetKinBody("torso_box")
            # add head
            self.addSphere("head_sphere", 0.4)
            self.obj_head_sphere = self.env.GetKinBody("head_sphere")
            while not rospy.is_shutdown():
                self.rolling = True
                if self.robot != None:
                    self.robot_rave_update_lock.acquire()
                    dof_values = self.robot.getAllDOFs()
                    self.robot_rave.SetDOFValues(dof_values)
                    self.robot_rave_update_lock.release()

                    # update head and torso
                    T_World_T2 = self.getLinkPose("torso_link2")
                    self.obj_torso_box.SetTransform(self.KDLToOpenrave(T_World_T2 * PyKDL.Frame(PyKDL.Vector(0, -0.40, 0))))
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

    def generateGrasps(self, target_name, show=False, checkcollision=True, checkik=True, checkgrasper=True):
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

        validgrasps,validindices = self.gmodel.computeValidGrasps(checkcollision=checkcollision, checkik=checkik, checkgrasper=checkgrasper)
        print "all valid grasps: %s"%(len(validgrasps))

        print "done."
        return validgrasps,validindices

    def getGraspTransform(self, grasp, collisionfree=False):
        return self.T_World_Br.Inverse()*self.OpenraveToKDL( self.gmodel.getGlobalGraspTransform(grasp,collisionfree=collisionfree) )

    def showGrasp(self, grasp):
        self.robot_rave_update_lock.acquire()
#        self.gmodel.showgrasp(grasp, collisionfree=True, useik=True)
        self.gmodel.showgrasp(grasp, collisionfree=False, useik=False)
        self.robot_rave_update_lock.release()

    def getGraspStandoff(self, grasp):
        return grasp[self.gmodel.graspindices.get('igraspstandoff')]

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

    def getFinalConfig(self, grasp):
        hand_config = None
        contacts = None
        self.robot_rave_update_lock.acquire()
        with self.robot_rave.CreateRobotStateSaver():
            with self.env:
                try:
                    contacts,finalconfig,mindist,volume = self.gmodel.runGraspFromTrans(grasp)
                    hand_config = [
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerOneKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerTwoKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerThreeKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerOneKnuckleOneJoint")],
                    ]
                except planning_error,e:
                    print "getFinalConfig: planning error"
        self.robot_rave_update_lock.release()
        if contacts == None:
            contacts_ret = None
        else:
            contacts_ret = []
            for c in contacts:
                contacts_ret.append(self.T_World_Br.Inverse() * PyKDL.Vector(c[0], c[1], c[2]))
        return hand_config, contacts_ret

    def grab(self, name):
#        body = self.env.GetKinBody(name)
#        self.robot_rave.Grab(body)
        self.robot_rave_update_lock.acquire()
        with self.env:
#            self.robot_rave.Grab( self.env.GetKinBody(name), self.right_palm_links_indices )
            self.robot_rave.Grab( self.env.GetKinBody(name))
        self.robot_rave_update_lock.release()

    def release(self, name):
        self.robot_rave_update_lock.acquire()
        with self.env:
            self.robot_rave.ReleaseAllGrabbed()
        self.robot_rave_update_lock.release()

#        self.robot_rave.RegrabAll()
#        body = self.env.GetKinBody(name)
#        self.env.Remove(body)
#        self.env.Add(body)

#        self.robot_rave.ResetGrabbed()
#        self.robot_rave.Grab( self.env.GetKinBody(name))
#        self.robot_rave.ReleaseAllGrabbed()
#        self.robot_rave.Release( self.env.GetKinBody(name))

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
#        return self.ikmodel.manip.FindIKSolution(IkParameterization(self.KDLToOpenrave(self.T_World_Br * T_Br_E),IkParameterizationType.Transform6D), 0)#IkFilterOptions.CheckEnvCollisions)
        return self.ikmodel.manip.FindIKSolution(self.KDLToOpenrave(self.T_World_Br * T_Br_E), IkFilterOptions.CheckEnvCollisions)

    def findIkSolutions(self, T_Br_E):
#        return self.ikmodel.manip.FindIKSolutions(IkParameterization(self.KDLToOpenrave(self.T_World_Br * T_Br_E),IkParameterizationType.Transform6D), 0)#IkFilterOptions.CheckEnvCollisions)
        return self.ikmodel.manip.FindIKSolutions(self.KDLToOpenrave(self.T_World_Br * T_Br_E), IkFilterOptions.CheckEnvCollisions)

    def planMove(self, T_Br_E, maxiter=500):
        traj = None
        self.robot_rave_update_lock.acquire()
        with self.robot_rave:
            try:
                self.robot_rave.SetActiveDOFs(self.right_arm_dof_indices)
                traj = self.basemanip.MoveToHandPosition(matrices=[self.KDLToOpenrave(self.T_World_Br * T_Br_E)],maxiter=maxiter,maxtries=1,seedik=4,execute=False,outputtrajobj=True)
            except planning_error,e:
                print "planMove: planning error"
        self.robot_rave_update_lock.release()

        if traj == None:
            return None

        conf = traj.GetConfigurationSpecification()
        q_prev = None
        max_q_vel = 0.0
        steps = max(2, int(traj.GetDuration()/0.01))
        traj_time_d = traj.GetDuration() / steps
        for t in np.linspace(0.0, traj.GetDuration(), steps):
            q = conf.ExtractJointValues(traj.Sample(t), self.robot_rave, self.right_arm_dof_indices)
            if q_prev != None:
                for i in range(0,7):
                    q_vel = math.fabs(q[i] - q_prev[i]) / traj_time_d
                    if q_vel > max_q_vel:
                        max_q_vel = q_vel
            q_prev = q

        time_d = 0.01
        q_vel_limit = 20.0/180.0*math.pi
        f = max_q_vel / (q_vel_limit)

        time = traj.GetDuration() * f
        times = []
        q_traj = []
        steps2 = max(2, int(f * traj.GetDuration() / 0.01))
        for t in np.linspace(0.0, traj.GetDuration(), steps2):
            q = conf.ExtractJointValues(traj.Sample(t), self.robot_rave, self.right_arm_dof_indices)
            q_traj.append(list(q))
            times.append(t/traj.GetDuration()*time)
        return q_traj, times

    def planMoveInJoints(self, q_dest):
        traj = None
        self.robot_rave_update_lock.acquire()
        with self.robot_rave:
            try:
                self.robot_rave.SetActiveDOFs(self.right_arm_dof_indices)
                traj = self.basemanip.MoveActiveJoints(goal=q_dest,execute=False,outputtrajobj=True)
            except planning_error,e:
                print "planMove: planning error"
        self.robot_rave_update_lock.release()

        if traj == None:
            return None

        conf = traj.GetConfigurationSpecification()
        q_prev = None
        max_q_vel = 0.0
        steps = max(2, int(traj.GetDuration()/0.01))
        traj_time_d = traj.GetDuration() / steps
        for t in np.linspace(0.0, traj.GetDuration(), steps):
            q = conf.ExtractJointValues(traj.Sample(t), self.robot_rave, self.right_arm_dof_indices)
            if q_prev != None:
                for i in range(0,7):
                    q_vel = math.fabs(q[i] - q_prev[i]) / traj_time_d
                    if q_vel > max_q_vel:
                        max_q_vel = q_vel
            q_prev = q

        time_d = 0.01
        q_vel_limit = 20.0/180.0*math.pi
        f = max_q_vel / (q_vel_limit)

        time = traj.GetDuration() * f
        times = []
        q_traj = []
        steps2 = max(2, int(f * traj.GetDuration() / 0.01))
        for t in np.linspace(0.0, traj.GetDuration(), steps2):
            q = conf.ExtractJointValues(traj.Sample(t), self.robot_rave, self.right_arm_dof_indices)
            q_traj.append(list(q))
            times.append(t/traj.GetDuration()*time)
        return q_traj, times

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


