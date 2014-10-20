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
from numpy import *
import numpy as np
import copy
import matplotlib.pyplot as plt
import thread
from scipy import optimize
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
        return None

#    def getObjectPose(self, obj):
#        for marker in obj.markers:
#            T_Br_M = self.getMarkerPose(marker[0], wait = False, timeBack = 0.3)
#            if T_Br_M != None:
#                T_Co_M = marker[1]
#                T_Br_Co = T_Br_M * T_Co_M.Inverse()
#                obj.updatePose(T_Br_Co)
        

    def poseUpdaterThread(self, args, *args2):
        index = 0
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.allow_update_objects_pose == None or not self.allow_update_objects_pose:
                continue
            for obj in self.objects:
                visible_markers_Br_Co = []
                for marker in obj.markers:
                    T_Br_M = self.getMarkerPose(marker[0], wait = False, timeBack = 0.3)
                    if T_Br_M != None and self.velma != None:
                        T_B_C = self.velma.T_B_C #self.getCameraPose()
                        T_C_M = T_B_C.Inverse() * T_Br_M
                        v = T_C_M * PyKDL.Vector(0,0,1) - T_C_M * PyKDL.Vector()
                        if v.z() > -0.7:
                            continue
                        T_Co_M = marker[1]
                        T_Br_Co = T_Br_M * T_Co_M.Inverse()
                        visible_markers_Br_Co.append(T_Br_Co)
                if len(visible_markers_Br_Co) > 0:
                    R_B_Co = velmautils.meanOrientation(visible_markers_Br_Co)[1]
                    p_B_Co = PyKDL.Vector()
                    for T_B_Co in visible_markers_Br_Co:
                        p_B_Co += T_B_Co.p
                    p_B_Co *= 1.0/float(len(visible_markers_Br_Co))
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

        # test joint impedance controll
        if False:
            # create the robot interface for real hardware
            velma = Velma()
            print "created robot interface"
            rospy.sleep(1.0)
            velma.switchToJoint()
            print "current q: %s"%(velma.qar)
            q = copy.deepcopy(velma.qar)
            q[6] += 0.2
            print "next q:    %s"%(q)
            raw_input("Press Enter to move the robot in joint in 5s...")
            velma.moveWristJoint(q, 1, None)
            rospy.sleep(1)
            exit(0)

        simulation_only = False
        if simulation_only:
            time_mult = 5.0
        else:
            time_mult = 8.0
        m_id = 0

        # create objects definitions
        obj_table = grip.GraspableObject("table", "box", [0.60,0.85,0.07])
#        obj_table.addMarker( 6, PyKDL.Frame(PyKDL.Vector(0, -0.225, 0.035)) )
        obj_table.addMarker( 6, PyKDL.Frame(PyKDL.Vector(0, 0.12, 0.035)) )

        obj_box = grip.GraspableObject("box", "box", [0.22,0.24,0.135])
        obj_box.addMarker( 7, PyKDL.Frame(PyKDL.Vector(-0.07, 0.085, 0.065)) )

        obj_big_box = grip.GraspableObject("big_box", "box", [0.20,0.20,2.0])
        obj_big_box.addMarker( 8, PyKDL.Frame(PyKDL.Vector(0, 0, 1.0)) )

        obj_wall_behind = grip.GraspableObject("wall_behind", "box", [0.20,3.0,3.0])
        obj_wall_right = grip.GraspableObject("wall_right", "box", [3.0,0.2,3.0])
        obj_ceiling = grip.GraspableObject("ceiling", "box", [3.0,3.0,0.2])

        obj_cbeam = grip.GraspableObject("cbeam", "cbeam", [0.1, 0.1, 0.2, 0.02])

        obj_model = "small_box"
        if obj_model == "small_box":
            obj_grasp = grip.GraspableObject("object", "box", [0.213, 0.056, 0.063])
            obj_grasp_frames = [
            [32, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1.0),PyKDL.Vector(-0.0,-0.0,-0.0))],
            [33, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.476415048088,0.506991531256,0.535847585538,0.478388601728),PyKDL.Vector(0.171902780644,0.015484630181,-0.0358638278243))],
            [34, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.00232981773607,0.00112909537834,0.00941735014758,0.999952304167),PyKDL.Vector(0.147645852213,0.000390249270086,0.000668615794865))],
            [35, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.00531442729247,-0.00148853793046,0.00813486006956,0.999951681417),PyKDL.Vector(0.0665568281854,0.00224423703193,0.00111350255405))],
            [36, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.693355695893,0.0176864851879,0.00475384182304,0.720362733769),PyKDL.Vector(0.0674437609483,-0.0264759261376,-0.0359458767302))],
            [37, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.697810893165,0.0185815546272,0.00926373682594,0.715981051696),PyKDL.Vector(0.14469056796,-0.0244927381799,-0.0402162367183))],
            [38, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.695822653251,0.00513123479502,0.00074687284782,0.718194923286),PyKDL.Vector(0.00246976816412,-0.0290920242125,-0.0345945703545))],
            [39, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.717544197129,-0.00121899527374,-0.00347930923759,0.696503218675),PyKDL.Vector(-0.000727941179089,0.0260474925053,-0.032006337674))],
            [40, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.714351406937,-0.00329260614077,-0.000231012293521,0.699779374364),PyKDL.Vector(0.143756457099,0.0313492884571,-0.0306638540963))],
            [41, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.723500552996,-0.0178604118324,0.00609666125085,0.690065783984),PyKDL.Vector(0.0672989279037,0.0311582168068,-0.0287569055643))],
            [42, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.999253045077,0.0186776422628,0.0309290299434,0.0137073954286),PyKDL.Vector(0.1452140182,0.00299751381556,-0.0731135644333))],
            [43, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.999700942127,0.0203100605493,-0.00475316283946,0.0127646070846),PyKDL.Vector(0.0671180998645,0.00225584347208,-0.0678502651338))],
            [44, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.99987100879,0.00361855348825,-0.0076843637788,0.0136316692101),PyKDL.Vector(9.73711128789e-05,-0.00034495011368,-0.0673206921801))],
            [45, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.697540042223,0.000607288852708,-0.71639447066,0.0147133647545),PyKDL.Vector(-0.0277395492321,-0.00467821975443,-0.0372505056897))],
            ]
            T_M32_M35 = obj_grasp_frames[3][1]
            T_M35_Co = PyKDL.Frame(PyKDL.Vector(0,0,-0.03))
            T_M32_Co = T_M32_M35 * T_M35_Co
            T_Co_M32 = T_M32_Co.Inverse()
            for marker in obj_grasp_frames:
                T_M32_Mi = marker[1]
                obj_grasp.addMarker(marker[0], T_Co_M32 * T_M32_Mi)

        elif obj_model == "big_box":
            obj_grasp = grip.GraspableObject("object", "box", [0.354, 0.060, 0.060])
            obj_grasp_frames = [
            [18, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1.0),PyKDL.Vector(-0.0,-0.0,-0.0))],
            [19, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.00165433105633,-0.00223969436551,0.00783500583865,0.999965429223),PyKDL.Vector(0.150364188592,0.00540315928786,0.00332539142516))],
            [20, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.00288819470565,0.000787875354111,-0.00584291384849,0.999978448739),PyKDL.Vector(0.283704103524,0.00072398461679,-0.00573581222652))],
            [21, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.700515458788,0.00669571919817,-0.000414650985238,0.71360569463),PyKDL.Vector(0.00448758480338,-0.0246391219393,-0.0318239341873))],
            [22, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.703637094621,0.0128037540093,-0.00696099928093,0.710410055845),PyKDL.Vector(0.147645037233,-0.0270235353887,-0.0319539994022))],
            [23, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.705573146762,-0.0108101497697,0.0078757141097,0.708510866789),PyKDL.Vector(0.2869132353,-0.0311870916024,-0.0364408741191))],
            [24, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.999936222986,-0.00719147497613,-0.00856953614561,0.00154780136503),PyKDL.Vector(0.000967154696901,-0.000658291054497,-0.059361255947))],
            [25, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.999925422492,0.00698873688352,-0.00978855330626,0.00211925234593),PyKDL.Vector(0.139811416338,-0.00107135691589,-0.0658641046354))],
            [26, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.999573485418,0.0127628877053,-0.0151291896644,0.0214723907838),PyKDL.Vector(0.294537733385,0.0266765305375,-0.0716188295568))],
            [27, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.715893512402,-0.00285901607723,0.00295541105269,0.698197372148),PyKDL.Vector(0.00499777040554,0.0411443197242,-0.0229397580848))],
            [28, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.720365484604,-0.00848358345308,-0.00122745492272,0.693541700807),PyKDL.Vector(0.153434321293,0.0483251803469,-0.017733228985))],
            [29, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.730806278242,-0.012226144189,-0.000233600920018,0.682475384546),PyKDL.Vector(0.299578008092,0.0554137486219,-0.0115267264344))],
            [30, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.00153631145759,-0.704471851921,-0.0141334264319,0.709589526314),PyKDL.Vector(-0.0328832398393,-0.000711552687509,-0.0280278186323))],
            [31, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.00648923236188,0.70344139916,-0.0037097168268,0.710713954987),PyKDL.Vector(0.320515478778,-0.000808849733968,-0.0336656231855))],
            ]
            T_M18_M19 = obj_grasp_frames[1][1]
            T_M19_Co = PyKDL.Frame(PyKDL.Vector(0,0,-0.03))
            T_M18_Co = T_M18_M19 * T_M19_Co
            T_Co_M18 = T_M18_Co.Inverse()
            for marker in obj_grasp_frames:
                T_M18_Mi = marker[1]
                obj_grasp.addMarker(marker[0], T_Co_M18 * T_M18_Mi)

        self.objects = [obj_table, obj_box, obj_grasp, obj_wall_behind, obj_wall_right, obj_ceiling]#, obj_cbeam]

        if False:
            grip.gripUnitTest(obj_grasp)
            exit(0)

        # load and init ik solver for right hand
        velma_ikr = velmautils.VelmaIkSolver()
        velma_ikr.initIkSolver()

        # simulation
        if simulation_only:
            self.getMarkerPose = self.getMarkerPoseFake

        # create Openrave interface
        self.openrave = openraveinstance.OpenraveInstance(PyKDL.Frame(PyKDL.Vector(0,0,0.1)))
        self.openrave.startNewThread()

        self.waitForOpenraveInit()

        print "openrave initialised"

        for obj in self.objects:
            if obj.isBox():
                self.openrave.addBox(obj.name, obj.size[0], obj.size[1], obj.size[2])
            elif obj.isCBeam():
                self.openrave.addCBeam(obj.name, obj.size[0], obj.size[1], obj.size[2], obj.size[3])

        print "added objects"

        self.openrave.updatePose("wall_behind", PyKDL.Frame(PyKDL.Vector(-0.5,0,1.5)) )
        self.openrave.updatePose("wall_right", PyKDL.Frame(PyKDL.Vector(0,-1.3,1.5)) )
        self.openrave.updatePose("ceiling", PyKDL.Frame(PyKDL.Vector(0,0,2.3)) )

        if False:
            index = 18
            for fr in frames:
                print index
                m_id = self.pub_marker.publishFrameMarker(fr, m_id)
                raw_input("Press Enter to continue...")
                rospy.sleep(0.1)
                index += 1
            rospy.sleep(2.0)

            exit(0)

        # unit test for surface sampling
        if False:
            vertices, indices = self.openrave.getMesh("object")
            velmautils.sampleMeshUnitTest(vertices, indices, self.pub_marker)

        self.velma = None
        self.allowUpdateObjects()
        # start thread for updating objects' positions in openrave
        thread.start_new_thread(self.poseUpdaterThread, (None,1))

        if simulation_only:
            # create the robot interface for simulation
            velma = VelmaSim(self.openrave, velma_ikr)
        else:
            # create the robot interface for real hardware
            velma = Velma()

        self.velma = velma
        self.openrave.addRobotInterface(velma)

        velma.updateTransformations()

        camera_fov_x = 50.0/180.0*math.pi
        camera_fov_y = 40.0/180.0*math.pi

#        self.openrave.setCamera(velma.T_B_C)

#        self.openrave.addCamera("head_camera",camera_fov_x, camera_fov_y, 0.2)
#        self.openrave.updatePose("head_camera", velma.T_B_C)

        k_pregrasp = Wrench(Vector3(1000.0, 1000.0, 1000.0), Vector3(300.0, 300.0, 300.0))
        k_grasp = Wrench(Vector3(500.0, 500.0, 500.0), Vector3(150.0, 150.0, 150.0))

        if True:
            # reset the gripper
            velma.resetFingers()
            velma.calibrateTactileSensors()
            velma.setMedianFilter(8)

            raw_input("Press Enter to enable cartesian impedance...")
            if velma.checkStopCondition():
                exit(0)
            velma.switchToCart()

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
            velma.moveImpedance(k_pregrasp, 3.0)
            if velma.checkStopCondition(3.0):
                exit(0)

        velma.updateTransformations()
        velma_init_T_B_W = copy.deepcopy(velma.T_B_W)

        if simulation_only:
#            velma.qar[1] += 10.0/180.0*math.pi
            velma.qar[6] = 90.0/180.0*math.pi
            rospy.sleep(1.0)

        sim_grips = []

        vertices, faces = self.openrave.getMesh("object")
        com_pt = velmautils.generateComSamples(vertices, faces, 2000)
        obj_grasp.addComPoints(com_pt)

        try:
            print "trying to read grasping data from file"
            with open('sim_grips_' + obj_model + '.txt', 'r') as f:
                for line in f:
                    if line == 'None' or line == 'None\n':
                        sim_grips.append(None)
                    else:
                        gr = grip.Grip(obj_grasp)
                        gr.fromStr(line)
                        sim_grips.append(gr)
        except IOError as e:
            print "could not read from file:"
            print e
            print "generating grapsing data..."
            self.disallowUpdateObjects()
            T_B_Oorig = self.openrave.getPose("object")
            # move the object 10 meters above
            self.openrave.updatePose("object", PyKDL.Frame(PyKDL.Vector(0,0,10)))
            T_B_O = self.openrave.getPose("object")
            T_O_B = T_B_O.Inverse()
            vertices, faces = self.openrave.getMesh("object")
            # get the possible grasps for the current scene

            valid_grasps = 0
            grips = []
            for idx in range(0, self.openrave.getGraspsCount("object")):
                sim_grips.append(None)
                grasp = self.openrave.getGrasp("object", idx)
                q, contacts = self.openrave.getFinalConfig("object", grasp)
                if contacts == None:
                    contacts = []
#                print "grasp_idx: %s   contacts: %s"%(idx, len(contacts))
                if len(contacts) == 0:
                    continue

                T_B_E = self.openrave.getGraspTransform("object", grasp)

                gr = grip.Grip(obj_grasp)
                self.pub_marker.eraseMarkers(2, m_id)
                m_id = 2

                checked_contacts = []
                contacts_groups = []
                while len(checked_contacts) < len(contacts):
                    pos = None
                    # group the contacts
                    for c_idx in range(0, len(contacts)):
                        if c_idx in checked_contacts:
                            continue
                        c_O = T_B_O.Inverse() * contacts[c_idx]
                        if pos == None:
                            pos = c_O
                            checked_contacts.append(c_idx)
                            contacts_groups.append([])
                            contacts_groups[-1].append(c_idx)
                        else:
                            dist = (pos - c_O).Norm()
                            if dist < 0.02:
                                checked_contacts.append(c_idx)
                                contacts_groups[-1].append(c_idx)

                centers = []
                for g in contacts_groups:
                    center = PyKDL.Vector()
                    for c in g:
                        center += T_B_O.Inverse() * contacts[c]
                    center *= 1.0/len(g)
                    centers.append(center)

                for c in centers:
                        points = velmautils.sampleMesh(vertices, faces, 0.003, [c], 0.01)
                        if len(points) == 0:
                            continue
                        # get the contact surface normal
                        fr = velmautils.estPlane(points)
                        # set the proper direction of the contact surface normal (fr.z axis)
                        fr_B_p = T_B_O * fr * PyKDL.Vector(0,0,0)
                        fr_B_z = PyKDL.Frame( copy.deepcopy((T_B_O * fr).M) ) * PyKDL.Vector(0,0,1)
                        # get the finger in contact index
                        P_F = PyKDL.Vector(0.0510813, -0.0071884, 0.0)
                        finger_idx_min = -1
                        d_min = 1000000.0
                        for finger_idx in range(0,3):
                            pt_B = T_B_E * velma.get_T_E_Fd( finger_idx, q[finger_idx], q[3]) * P_F
                            d = (fr_B_p - pt_B).Norm()
                            if d < d_min:
                                d_min = d
                                finger_idx_min = finger_idx
#                        print "finger_idx_min: %s"%(finger_idx_min)
                        n_B = PyKDL.Frame( copy.deepcopy((T_B_E * velma.get_T_E_Fd( finger_idx_min, q[finger_idx_min], q[3])).M) ) * PyKDL.Vector(1,-1,0)
                        if PyKDL.dot(n_B, fr_B_z) < 0:
                            fr = fr * PyKDL.Frame(PyKDL.Rotation.RotX(math.pi))
                        # add the contact to the grip description
                        gr.addContact(fr)

                valid_grasps += 1
                sim_grips[-1] = gr

            print "added grasps: %s / %s"%(valid_grasps, len(sim_grips))
            print "writing grasping data to file"
            with open('sim_grips_' + obj_model + '.txt', 'w') as f:
                for gr in sim_grips:
                    if gr == None:
                        f.write('None\n')
                    else:
                        f.write(gr.toStr() + '\n')
            self.openrave.updatePose("object", T_B_Oorig)
            self.allowUpdateObjects()

        if False:
            velmautils.comSamplesUnitTest(self.openrave, self.pub_marker, "cbeam")
            exit(0)

        if False:
            velmautils.updateComUnitTest(self.openrave, self.pub_marker, "object")
            exit(0)

        ################
        # the main loop
        ################
        base_qar = copy.deepcopy(velma.qar)
        T_B_O_prev = None
        checked_grasps_idx = []
        grips_db = []
        while True:

            self.allowUpdateObjects()
            rospy.sleep(1.0)
            # move to base pose
            dist = 0.0
            for q_idx in range(0,7):
                dist += (velma.qar[q_idx] - base_qar[q_idx])*(velma.qar[q_idx] - base_qar[q_idx])
            if math.sqrt(dist) > 10.0/180.0*math.pi:
                print "moving to base pose..."
                traj = self.openrave.planMoveForRightArm(None, base_qar)
                if traj == None:
                    print "FATAL ERROR: colud not plan trajectory to base pose"
                    return

                duration = math.fsum(traj[3])

                raw_input("Press Enter to visualize the trajectory...")
                if velma.checkStopCondition():
                    exit(0)
                self.openrave.showTrajectory(duration * time_mult * 0.3, qar_list=traj[4])

                self.switchToJoint(velma)

                print "trajectory len: %s"%(len(traj[0]))
                raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
                if velma.checkStopCondition():
                    exit(0)
                velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                if velma.checkStopCondition(duration * time_mult + 1.0):
                    exit(0)

            velma.calibrateTactileSensors()

            rospy.sleep(2.0)
            self.disallowUpdateObjects()
            rospy.sleep(0.2)

            print "generating possible grasps..."
            # check if we need to generate new set of possible grasps
            generate_new_grasps = True
            T_B_O = self.openrave.getPose("object")
            if T_B_O_prev != None:
                T_B_O_diff = PyKDL.diff(T_B_O_prev, T_B_O)
                if T_B_O_diff.vel.Norm() < 0.01 and T_B_O_diff.rot.Norm() < 3.0/180.0*math.pi:
                    generate_new_grasps = False
            T_B_O_prev = T_B_O
            if generate_new_grasps:
                # get the possible grasps for the current scene
                indices = self.openrave.generateGrasps("object")

            if len(indices) == 0:
                print "FATAL ERROR: could not generate any grasps for current configuration"
                exit(0)

            print "number of possible grasps for current pose: %s"%(len(indices))

            # show all possible grasps
#            for idx in indices:
#                self.openrave.showGrasp("object", self.openrave.getGrasp("object", idx))

            max_dist = -1000000.0
            min_score = None
            max_idx = None
            # TODO
            first_ok_idx = None
            # iterate through all available grasps
            for idx in indices:
                if sim_grips[idx] == None:
                    continue
                # ignore grasps which failed due to planning error for pose close to the current pose
                close_to_failed = False
                for T_Br_O_failed in sim_grips[idx].planning_failure_poses:
                    diff = PyKDL.diff(obj_grasp.T_Br_Co, T_Br_O_failed)
                    if diff.vel.Norm() < 0.02 and diff.rot.Norm() < 5.0/180.0*math.pi:
                        close_to_failed = True
                        break
                if close_to_failed:
                    continue

                # ignore grasps which failed due to visibility error for pose close to the current pose
                close_to_failed = False
                for T_Br_O_failed in sim_grips[idx].visibility_problem_poses:
                    diff = PyKDL.diff(obj_grasp.T_Br_Co, T_Br_O_failed)
                    if diff.vel.Norm() < 0.02 and diff.rot.Norm() < 5.0/180.0*math.pi:
                        close_to_failed = True
                        break
                if close_to_failed:
                    continue

                # ignore the grasps with:
                # count_no_contact > 0
                # count_too_little_contacts > 0
                # count_moved_on_grip > 0
                # count_unstable > 0
                # count_stable > 0
                if sim_grips[idx].count_no_contact > 0 or sim_grips[idx].count_too_little_contacts > 0 or sim_grips[idx].count_moved_on_grip > 0 or sim_grips[idx].count_unstable > 0 or sim_grips[idx].count_stable > 0:
                    continue

                # save the index of the first possibly good grasp
                if first_ok_idx == None:
                    first_ok_idx = idx

                # search for grasp most distant from other grasps with:
                # count_no_contact > 0
                # count_too_little_contacts > 0
                # count_unstable > 0

                # iterate through all checked grasps and calculate the total score
                score = 0.0
                for idx_2 in range(0, self.openrave.getGraspsCount("object")):
                    if sim_grips[idx_2] == None:
                        continue
                    sc_mul = 1.0
                    if sim_grips[idx_2].count_no_contact == 0 and sim_grips[idx_2].count_too_little_contacts == 0 and sim_grips[idx_2].count_unstable == 0 and sim_grips[idx_2].count_stable == 0:
                        continue
                    dist, angles, all_scores, all_angles, all_scores2, n1_s_list, pos1_s_list, n2_s_list, pos2_s_list = grip.gripDist3(sim_grips[idx], sim_grips[idx_2])

                    penalty_no_contact = 4.0 * sim_grips[idx_2].count_no_contact * max(5.0-dist, 0.0)
                    penalty_too_little_contacts = 4.0 * sim_grips[idx_2].count_too_little_contacts * max(5.0-dist, 0.0)
                    penalty_unstable = sim_grips[idx_2].count_unstable * max(5.0-dist, 0.0)
                    reward_stable = sim_grips[idx_2].count_stable * max(5.0-dist, 0.0)

                    score += penalty_no_contact + penalty_too_little_contacts + penalty_unstable - reward_stable

                if min_score == None or min_score > score:
                    min_score = score
                    max_idx = idx

#                min_dist = None
#                # iterate through all checked grasps - find the closest grasp in checked grasps set
#                for idx_2 in range(0, self.openrave.getGraspsCount("object")):
#                    if sim_grips[idx_2] == None:
#                        continue
#                    if sim_grips[idx_2].count_no_contact == 0 and sim_grips[idx_2].count_too_little_contacts == 0 and sim_grips[idx_2].count_unstable == 0:
#                        continue
#                    dist, angles, all_scores, all_angles, all_scores2, n1_s_list, pos1_s_list, n2_s_list, pos2_s_list = grip.gripDist2(sim_grips[idx], sim_grips[idx_2])
#                    if min_dist == None or dist < min_dist:
#                        min_dist = dist
#                if min_dist != None and min_dist > max_dist:
#                    max_dist = min_dist
#                    max_idx = idx
#            print "found grasp that is the most distant from all checked grasps: %s"%(max_dist)
            print "found grasp that has the best score: %s"%(min_score)

            if max_idx != None:
                grasp_idx = max_idx
            elif first_ok_idx != None:
                grasp_idx = first_ok_idx
            else:
                print "FATAL ERROR: max_idx == None and first_ok_idx == None"
                exit(0)

            current_sim_grip = sim_grips[grasp_idx]

            print "choosed grasp no. %s"%(grasp_idx)

            print "found grasp"
            grasp = copy.deepcopy(self.openrave.getGrasp("object", grasp_idx))

            T_B_Ed = self.openrave.getGraspTransform("object", grasp, collisionfree=True)
            self.openrave.showGrasp("object", grasp)

            T_Br_O_init = obj_grasp.T_Br_Co
            traj = self.openrave.planMoveForRightArm(T_B_Ed, None)
            if traj == None:
                print "colud not plan trajectory"
                current_sim_grip.setPlanningFailure(obj_grasp.T_Br_Co)
                continue

            final_config, contacts = self.openrave.getFinalConfig("object", grasp)
            if final_config == None:
                print "colud not plan trajectory"
                current_sim_grip.setPlanningFailure(obj_grasp.T_Br_Co)
                continue

            print "final_config:"
            print final_config
            print "contacts (sim): %s"%(len(contacts))
            print "standoff: %s"%(self.openrave.getGraspStandoff("object", grasp))

#            print "q_start: %s"%(traj[0][0])
#            print "q_end:   %s"%(traj[0][-1])

            duration = math.fsum(traj[3])

            raw_input("Press Enter to visualize the trajectory...")
            if velma.checkStopCondition():
                exit(0)
            self.openrave.showTrajectory(duration * time_mult * 0.3, qar_list=traj[4])

            self.switchToJoint(velma)

            print "trajectory len: %s"%(len(traj[0]))
            raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(duration * time_mult + 1.0):
                exit(0)

            self.switchToCartesian(velma)

            # move to the desired position
            velma.updateTransformations()
            T_B_Wd = T_B_Ed * velma.T_E_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
            raw_input("Press Enter to move the robot in " + str(duration) + " s...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

            raw_input("Press Enter to close fingers for pre-grasp...")
            # close the fingers for pre-grasp
            ad = 5.0/180.0*math.pi
            velma.move_hand_client([final_config[0]-ad, final_config[1]-ad, final_config[2]-ad, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))

            print "setting stiffness to lower value"
            velma.moveImpedance(k_grasp, 3.0)
            if velma.checkStopCondition(3.0):
                break

            # close the fingers
            ad2 = 20.0/180.0*math.pi
            velma.move_hand_client([final_config[0]+ad2, final_config[1]+ad2, final_config[2]+ad2, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(2500.0, 2500.0, 2500.0, 2500.0))
            if velma.checkStopCondition(3.0):
                break

            if not simulation_only:
                # get contact points and forces for each finger
                velma.updateTransformations()
                contacts = [[],[],[]]
                forces = [[],[],[]]
                contacts[0], forces[0] = velma.getContactPoints(100, f1=True, f2=False, f3=False, palm=False)
                contacts[1], forces[1] = velma.getContactPoints(100, f1=False, f2=True, f3=False, palm=False)
                contacts[2], forces[2] = velma.getContactPoints(100, f1=False, f2=False, f3=True, palm=False)
                fingers_in_contact = 0
                print "f1: %s   %s    %s"%((final_config[0]+ad2), velma.qhr[1], len(contacts[0]))
                print "f2: %s   %s    %s"%((final_config[1]+ad2), velma.qhr[2], len(contacts[1]))
                print "f3: %s   %s    %s"%((final_config[2]+ad2), velma.qhr[3], len(contacts[2]))
                if abs((final_config[0]+ad2) - velma.qhr[1]) > 1.0/180.0*math.pi or len(contacts[0]) > 0:
                    fingers_in_contact += 1
                    f1_contact = True
                if abs((final_config[1]+ad2) - velma.qhr[2]) > 1.0/180.0*math.pi or len(contacts[1]) > 0:
                    fingers_in_contact += 1
                    f2_contact = True
                if abs((final_config[3]+ad2) - velma.qhr[3]) > 1.0/180.0*math.pi or len(contacts[2]) > 0:
                    fingers_in_contact += 1
                    f3_contact = True
            else:
                fingers_in_contact = 3

            if fingers_in_contact == 0:
                current_sim_grip.setNoContact()
            elif fingers_in_contact < 3:
                current_sim_grip.setTooLittleContacts()

            if fingers_in_contact < 3:
                print "only %s fingers have contact"%(fingers_in_contact)
                self.openrave.release("object")
                velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
                if velma.checkStopCondition(3.0):
                    break
                continue

            self.allowUpdateObjects()
            print "checking the object pose after grip..."
            rospy.sleep(1.0)

            # grab the body
            self.openrave.grab("object")

            pose_tolerance = [0.02, 5.0/180.0*math.pi]
            # check the object pose before lift-up
            dur = rospy.Time.now() - obj_grasp.pose_update_time
            if abs(dur.to_sec()) < 1.0:
                print "fresh pose available: %s"%(dur.to_sec())
                fresh_pose = True
                T_Br_Co_sim = self.openrave.getPose("object")
                pose_diff = PyKDL.diff(obj_grasp.T_Br_Co, T_Br_Co_sim)
                if pose_diff.vel.Norm() > pose_tolerance[0] or pose_diff.rot.Norm() > pose_tolerance[1]:
                    print "object pose is different after the grip - diff: %s > %s     %s > %s deg. but it is okay..."%(pose_diff.vel.Norm(), pose_tolerance[0], pose_diff.rot.Norm()/math.pi*180.0, pose_tolerance[1]/math.pi*180.0)
#                    current_sim_grip.setMovedOnGrip()
#                    self.openrave.release("object")
#                    velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
#                    if velma.checkStopCondition(3.0):
#                        break
#                    continue
            else:
                print "fresh pose not available: %s"%(dur.to_sec())
                fresh_pose = False

            # lift the object up
            velma.updateTransformations()

            T_B_Ebeforelift = velma.T_B_W * velma.T_W_E
            T_B_Wd = PyKDL.Frame(PyKDL.Vector(0,0,0.08)) * velma.T_B_W
            # save the initial position after lift up
            T_B_Eafterlift = T_B_Wd * velma.T_W_E

            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.02, max_v_r=0.04)
            raw_input("Press Enter to move the robot in " + str(duration) + " s...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

            qar_after_lift = copy.deepcopy(velma.qar)

            # get contact points and forces for each finger
            if not simulation_only:
                velma.updateTransformations()
                contacts = [[],[],[]]
                forces = [[],[],[]]
                contacts[0], forces[0] = velma.getContactPoints(100, f1=True, f2=False, f3=False, palm=False)
                contacts[1], forces[1] = velma.getContactPoints(100, f1=False, f2=True, f3=False, palm=False)
                contacts[2], forces[2] = velma.getContactPoints(100, f1=False, f2=False, f3=True, palm=False)
                fingers_in_contact = 0
                if len(contacts[0]) > 0:
                    fingers_in_contact += 1
                if len(contacts[1]) > 0:
                    fingers_in_contact += 1
                if len(contacts[2]) > 0:
                    fingers_in_contact += 1
            else:
                fingers_in_contact = 3

            if fingers_in_contact < 2:
                print "only %s fingers have contact"%(fingers_in_contact)
                current_sim_grip.setUnstable()
                self.openrave.release("object")
                velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
                if velma.checkStopCondition(3.0):
                    break
                continue

            self.allowUpdateObjects()
            print "checking the object pose after the lift-up..."
            rospy.sleep(1.0)
            # check the object pose before lift-up
            dur = rospy.Time.now() - obj_grasp.pose_update_time
            if abs(dur.to_sec()) < 1.0:
                print "fresh pose available: %s"%(dur.to_sec())
                fresh_pose = True
                T_Br_Co_sim = self.openrave.getPose("object")
                pose_diff = PyKDL.diff(obj_grasp.T_Br_Co, T_Br_Co_sim)
                if pose_diff.vel.Norm() > pose_tolerance[0] or pose_diff.rot.Norm() > pose_tolerance[1]:
                    print "object pose is different after the lift-up - diff: %s > %s     %s > %s deg."%(pose_diff.vel.Norm(), pose_tolerance[0], pose_diff.rot.Norm()/math.pi*180.0, pose_tolerance[1]/math.pi*180.0)
                    print "adding experience for determination of COM"
                    # calculate the contacts in object frame
                    contacts_O = []
                    for c in list(contacts[0]) + list(contacts[1]) + list(contacts[2]):
                        contacts_O.append(obj_grasp.T_Br_Co.Inverse() * c)

                    m_id = 0
                    T_B_O = T_Br_O_init
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.354, 0.060, 0.060), T=T_B_O)
                    m_id = self.pub_marker.publishMultiPointsMarker(contacts_O, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=T_B_O)

                    T_B_O_2 = obj_grasp.T_Br_Co
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.354, 0.060, 0.060), T=T_B_O_2)
                    m_id = self.pub_marker.publishMultiPointsMarker(contacts_O, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=T_B_O_2)

                    m_id = obj_grasp.updateCom(T_Br_O_init ,obj_grasp.T_Br_Co, contacts_O, m_id=m_id, pub_marker=self.pub_marker)
                    # get max value
                    max_com = None
                    for com_value in obj_grasp.com_weights:
                        if max_com == None or max_com < com_value:
                            max_com = com_value
                    good_com_count = 0
                    for idx in range(0, len(obj_grasp.com_pt)):
                        if obj_grasp.com_weights[idx] == max_com:
                            good_com_count += 1
                            m_id = pub_marker.publishSinglePointMarker(obj_grasp.com_pt[idx], m_id, r=0, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=obj_grasp.T_Br_Co)
                        else:
                            m_id = pub_marker.publishSinglePointMarker(obj_grasp.com_pt[idx], m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=obj_grasp.T_Br_Co)
                        if idx % 10 == 0:
                            rospy.sleep(0.01)
                    print "COM estimation: %s  com: %s"%(float(good_com_count)/float(len(obj_grasp.com_pt)), obj_grasp.com)

                    current_sim_grip.setUnstable()
                    self.openrave.release("object")
                    velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
                    if velma.checkStopCondition(3.0):
                        break
                    continue
            else:
                print "fresh pose not available: %s"%(dur.to_sec())
                fresh_pose = False
                current_sim_grip.setVisibilityProblem(T_Br_O_init)
                self.openrave.release("object")
                velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
                if velma.checkStopCondition(3.0):
                    break
                continue

            print "success"
            current_sim_grip.setStable()
            self.openrave.release("object")
            velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
            if velma.checkStopCondition(3.0):
                break
            rospy.sleep(1.0)

            continue












#            raw_input("Press Enter to exit...")
#            exit(0)


            print "checking the orientation of the object..."
            # TODO
            stable_grasp = True

            # basic orientations of the gripper, we can rotate them in global z axis and move them around
            main_R_Br_E2 = [
            PyKDL.Frame(),                                           # gripper points up
            PyKDL.Frame(PyKDL.Rotation.RotX(180.0/180.0*math.pi)),    # gripper points down
            PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi)),     # gripper points right (E.y points up)
            PyKDL.Frame(PyKDL.Rotation.RotX(-90.0/180.0*math.pi)),    # gripper points left (E.y points down)
            PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi)),     # gripper points to front (E.x points down)
            PyKDL.Frame(PyKDL.Rotation.RotY(-90.0/180.0*math.pi)),    # gripper points to back (E.x points up)
            ]
            main_R_Br_E = [
            [PyKDL.Frame(),1],                                           # gripper points up
            [PyKDL.Frame(PyKDL.Rotation.RotX(180.0/180.0*math.pi)),0],    # gripper points down
            [PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi)),3],     # gripper points right (E.y points up)
            [PyKDL.Frame(PyKDL.Rotation.RotX(-90.0/180.0*math.pi)),2],    # gripper points left (E.y points down)
            [PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi)),5],     # gripper points to front (E.x points down)
            [PyKDL.Frame(PyKDL.Rotation.RotY(-90.0/180.0*math.pi)),4],    # gripper points to back (E.x points up)
            ]

            vis = self.openrave.getVisibility("object", velma.T_B_C, pub_marker=None, fov_x=camera_fov_x, fov_y=camera_fov_y, min_dist=0.2)
            print "vis: %s"%(vis)

            self.openrave.printCollisions()
            wrist_col = velmautils.WristCollisionAvoidance("right", None, 5.0/180.0*math.pi)
            velma.updateTransformations()
            list_T_B_Ed = []
            singularity_angle = 20.0/180.0*math.pi
            for basic_orientation_idx in range(0, len(main_R_Br_E)):
                if not stable_grasp:
                    break
                R_Br_E = main_R_Br_E[basic_orientation_idx][0]
                print "generate reasonable destinations for one basic orientation no. %s"%(basic_orientation_idx)
                # generate reasonable destinations for one basic orientation
                list_T_B_Ed.append([])
                angle_steps = 10
                found_solution = False
                for r in np.linspace(0.0, 0.2, 5):
                    density = 10.0    # per meter
#                    L = 2.0 * math.pi * r
                    if r == 0.0 or 1.0/(r*density) > math.pi:
                        v_sphere = velmautils.generateNormalsSphere(math.pi)
                    else:
                        v_sphere = velmautils.generateNormalsSphere(1.0/(r*density))
                    print "normals: %s"%(len(v_sphere))
                    for angle_deg in np.linspace(0.0, 360.0*(float(angle_steps-1)/angle_steps), angle_steps):
                        # calculate orientation (rotate along global z axis)
                        R_Br_Ed = PyKDL.Frame(PyKDL.Rotation.RotZ(angle_deg/180.0*math.pi)) * R_Br_E
                        # set the position
                        T_E_G = PyKDL.Frame(PyKDL.Vector(0,0,0.2))
                        T_G_E = T_E_G.Inverse()
                        # iterate gripper position in camera frame:

                        A = 4.0 * math.pi * r * r
                        density = 100.0    # per square meter
                        steps = int(A * density) + 1
                        for i in range(0, steps):
                            v = r * v_sphere[random.randint(0, len(v_sphere)-1)]#PyKDL.Frame( PyKDL.Rotation.RotX(random.uniform(-math.pi, math.pi)) * PyKDL.Rotation.RotY(random.uniform(-math.pi, math.pi))) * PyKDL.Vector(0,0,r)
                            pt_G_in_B = velma.T_B_C * PyKDL.Vector(0, 0, 0.5) + v
                            T_B_Gd = PyKDL.Frame(copy.deepcopy(R_Br_Ed.M), pt_G_in_B)
                            T_B_Ed = T_B_Gd * T_G_E
                            q_out = self.openrave.findIkSolution(T_B_Ed)
                            if q_out != None:
                                if len(wrist_col.getQ5Q6SpaceSectors(q_out[5],q_out[6])) > 0:
                                    vis = self.openrave.getVisibility("object", velma.T_B_C, qar=q_out, pub_marker=None, fov_x=camera_fov_x, fov_y=camera_fov_y, min_dist=0.2)
#                                    print "vis: %s   i: %s   angle: %s    r: %s    o: %s"%(vis, i, angle_deg, r, basic_orientation_idx)
                                    if vis > 0.8:
                                        found_solution = True
                                        list_T_B_Ed[-1].append( [T_B_Ed, q_out] )
                                        print "i: %s"%(i)
                                        break
#                            else:
#                                print "q_out == None"
                        if found_solution:
                            print "angle_deg: %s"%(angle_deg)
                            break
                    if found_solution:
                        print "r: %s"%(r)
                        break
                print "generated %s poses"%(len(list_T_B_Ed[-1]))

            checked_orientations = []

            raw_input("Press Enter to enable joint impedance...")
            if velma.checkStopCondition():
                exit(0)
            velma.switchToJoint()

            while len(checked_orientations) < len(main_R_Br_E):
                if not stable_grasp:
                    break

                velma.updateTransformations()

                # iterate through all poses and get the closest with reasonable visibility
                print "looking for reachable and visible poses..."
                min_cost = 1000000.0
                best_basic_orientation_idx = -1
                best_pose_idx = -1
                best_q_dest = None
                best_traj = None
                for basic_orientation_idx in range(0, len(main_R_Br_E)):
                    if basic_orientation_idx in checked_orientations:
                        continue
                    for pose_idx in range(0, len(list_T_B_Ed[basic_orientation_idx])):
                            T_B_Ed = list_T_B_Ed[basic_orientation_idx][pose_idx][0]
                            q_dest = list_T_B_Ed[basic_orientation_idx][pose_idx][1]

                            cost = 0.0
                            for q_idx in range(0, 7):
                                cost += (velma.qar[q_idx]-q_dest[q_idx])*(velma.qar[q_idx]-q_dest[q_idx])
                            if velma.qar[3]*q_dest[3] < 0:
                                cost += 100.0
                            if cost < min_cost:
                                min_cost = cost
                                best_basic_orientation_idx = basic_orientation_idx
                                best_pose_idx = pose_idx
                                best_q_dest = q_dest
                                best_traj = traj

                print "best_basic_orientation_idx: %s best_pose_idx: %s   min_cost: %s"%(best_basic_orientation_idx, best_pose_idx, min_cost)

#                self.openrave.showTrajectory(3.0, qar_list=[best_q_dest, best_q_dest])
                if best_pose_idx < 0:
                    print "could not find next pose"
                    break

                T_B_Ed = list_T_B_Ed[best_basic_orientation_idx][best_pose_idx][0]
                q_dest = list_T_B_Ed[best_basic_orientation_idx][best_pose_idx][1]
                traj = self.openrave.planMoveForRightArm(None, q_dest)
                if traj != None:
                    self.openrave.showTrajectory(duration * time_mult * 0.5, qar_list=traj[4])
                    duration = math.fsum(traj[3])
                    raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
                    if velma.checkStopCondition():
                        exit(0)
                    velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                    if velma.checkStopCondition(duration * time_mult + 1.0):
                        exit(0)
                else:
                    print "colud not plan trajectory"
                    rospy.sleep(4.0)
                    exit(0)

#                self.executeTrajectoryInOneSubspace(T_Br_Ed, velma, velma_ikr)
                vis = self.openrave.getVisibility("object", velma.T_B_C, pub_marker=None, fov_x=camera_fov_x, fov_y=camera_fov_y, min_dist=0.2)
                print "reached the desired pose. Visibility: %s"%(vis)

                checked_orientations.append(best_basic_orientation_idx)
                if best_basic_orientation_idx == 0:
                    checked_orientations.append(1)
                if best_basic_orientation_idx == 1:
                    checked_orientations.append(0)
                if best_basic_orientation_idx == 2:
                    checked_orientations.append(3)
                if best_basic_orientation_idx == 3:
                    checked_orientations.append(2)
                if best_basic_orientation_idx == 4:
                    checked_orientations.append(5)
                if best_basic_orientation_idx == 5:
                    checked_orientations.append(4)
                print "checked_orientations: %s"%(checked_orientations)

                print "checking the orientation of the object..."
                # TODO
                stable_grasp = True

            if stable_grasp:
                print "moving to initial pose"
                traj = self.openrave.planMoveForRightArm(None, qar_after_lift)
                if traj != None:
                    self.openrave.showTrajectory(duration * time_mult * 0.5, qar_list=traj[4])
                    duration = math.fsum(traj[3])
                    raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
                    if velma.checkStopCondition():
                        exit(0)
                    velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                    if velma.checkStopCondition(duration * time_mult + 1.0):
                        exit(0)
                else:
                    print "colud not plan trajectory"
                    rospy.sleep(4.0)
                    exit(0)

                raw_input("Press Enter to enable cartesian impedance...")
                if velma.checkStopCondition():
                    exit(0)
                velma.switchToCart()

                T_B_Wd = T_B_Ebeforelift * velma.T_E_W
                duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
                velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                if velma.checkStopCondition(duration):
                    break
            else:
                # TODO: check the position of the object and decide what to do
                pass

            velma.move_hand_client([0, 0, 0, 0], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))

            print "releasing the body"
            # release the body
            if simulation_only: 
                self.openrave.release("object")
            else:
                pass

            self.allowUpdateObjects()

            raw_input("Press Enter to enable cartesian impedance...")
            if velma.checkStopCondition():
                exit(0)
            velma.switchToCart()

            print "moving the gripper up"
            T_B_Wd = T_B_Eafterlift * velma.T_E_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break


#            rospy.sleep(2.0)

#            raw_input("Press Enter to exit...")
#            exit(0)
            continue
















            raw_input("Press Enter to close fingers for grasp...")

            # close the fingers for grasp
            velma.move_hand_client((120.0/180.0*math.pi,120.0/180.0*math.pi,120.0/180.0*math.pi,0), v=(1.2, 1.2, 1.2, 1.2), t=(1500.0, 1500.0, 1500.0, 1500.0))
            m_id = 0
            if True:
                if velma.checkStopCondition(3.0):
                    break
            else:
                time_end = rospy.Time.now() + rospy.Duration(3.0)
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
                        break
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
            number_of_fingers_in_contact = 0
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
                    number_of_fingers_in_contact += 1
                else:
                    centers.append(None)

            print "fingers in contact: %s"%(number_of_fingers_in_contact)
            if number_of_fingers_in_contact < 2:
                print "could not grasp the object with more than 1 finger"
                break

            gr = grip.Grip(obj_grasp)

            for c in centers:
                if c != None:
                    m_id = self.pub_marker.publishSinglePointMarker(c, m_id, r=1, g=1, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))

            T_Br_O = obj_grasp.T_Br_Co

            T_E_Co_before = velma.T_E_W * velma.T_B_W.Inverse() * T_Br_O

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
                    # get the contact surface normal
                    fr = velmautils.estPlane(points)
                    # set the proper direction of the contact surface normal (fr.z axis)
                    if PyKDL.dot( T_Br_O * fr * PyKDL.Vector(0,0,1), velma.T_B_W * velma.T_W_E * T_E_Fi3[finger] * PyKDL.Vector(1,-1,0) ) > 0:
                        fr = fr * PyKDL.Frame(PyKDL.Rotation.RotX(180.0/180.0*math.pi))
                    # add the contact to the grip description
                    gr.addContact(fr)
                    m_id = self.pub_marker.publishFrameMarker(T_Br_O*fr, m_id)
                    rospy.sleep(1.0)
                finger += 1

            self.allowUpdateObjects()

            # lift the object up
            velma.updateTransformations()

            T_B_Wd = PyKDL.Frame(PyKDL.Vector(0,0,0.05)) * velma.T_B_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
            velma.moveWrist2(T_B_Wd*velma.T_W_T)
            raw_input("Press Enter to lift the object up in " + str(duration) + " s...")
            if velma.checkStopCondition():
                break
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

            if velma.checkStopCondition(2.0):
                break

            contacts, forces = velma.getContactPoints(200, f1=True, f2=True, f3=True, palm=False)
            if len(contacts) > 0:
                print "Still holding the object. Contacts: %s"%(len(contacts))
                holding = True
            else:
                holding = False

            # try to get fresh object pose
            dur = rospy.Time.now() - obj_grasp.pose_update_time
            if dur.to_sec() < 1.0:
                fresh_pose = True
            else:
                fresh_pose = False

            velma.updateTransformations()
            if fresh_pose:
                print "we can still see the object!"
                T_E_Co_after = velma.T_E_W * velma.T_B_W.Inverse() * obj_grasp.T_Br_Co
                T_E_Co_diff = PyKDL.diff(T_E_Co_before, T_E_Co_after)
                print "T_E_Co_diff: %s"%(T_E_Co_diff)
            else:
                print "we can't see the object!"

            gr.success()

            grips_db.append( gr )

            raw_input("Press Enter to open fingers...")

            velma.move_hand_client((0,0,0,0), v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
            if velma.checkStopCondition(3.0):
                break

            duration = velma.getMovementTime(velma_init_T_B_W, max_v_l=0.1, max_v_r=0.2)
            velma.moveWrist2(velma_init_T_B_W*velma.T_W_T)
            raw_input("Press Enter to move back to initial position in " + str(duration) + " s...")
            if velma.checkStopCondition():
                break
            velma.moveWrist(velma_init_T_B_W, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

        # grasping loop end

        for g in grips_db:
            g.serializePrint()

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


