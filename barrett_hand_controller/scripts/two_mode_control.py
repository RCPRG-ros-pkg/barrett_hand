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
                for marker in obj.markers:
                    T_Br_M = self.getMarkerPose(marker[0], wait = False, timeBack = 0.3)
                    if T_Br_M != None:
                        T_B_C = self.getCameraPose()
                        T_C_M = T_B_C.Inverse() * T_Br_M
                        v = T_C_M * PyKDL.Vector(0,0,1) - T_C_M * PyKDL.Vector()
                        if v.z() > -0.7:
                            continue
                        T_Co_M = marker[1]
                        T_Br_Co = T_Br_M * T_Co_M.Inverse()
                        obj.updatePose(T_Br_Co)
                        self.openrave.updatePose(obj.name, T_Br_Co)
                        break

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
            time_mult = 10.0
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

        obj_grasp = grip.GraspableObject("object", "box", [0.354, 0.060, 0.060])
        obj_grasp_frames_old = [
        [18, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1.0),PyKDL.Vector(-0.0,-0.0,-0.0))],
        [19, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.00785118489648,-0.00136981350282,-0.000184602454162,0.999968223709),PyKDL.Vector(0.14748831582,-0.00390004064458,0.00494675382036))],
        [20, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.0108391070454,-0.00679278400361,-0.0154191552083,0.999799290606),PyKDL.Vector(0.289969171073,-0.00729932931459,0.00759828464719))],
        [21, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.707914450157,0.00553703354292,-0.0049088621984,0.706259425134),PyKDL.Vector(-0.00333471065688,-0.0256403932819,-0.0358967610179))],
        [22, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.711996124932,0.000529252451241,-0.00578615630039,0.702159353971),PyKDL.Vector(0.147443644368,-0.03209918445,-0.028549100504))],
        [23, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.714618336612,-0.00917868744082,0.000177822438207,0.699454325209),PyKDL.Vector(0.29031370529,-0.0348959795876,-0.0263138015496))],
        [24, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.999814315554,-0.00730751409695,0.00318617054665,0.0175437444253),PyKDL.Vector(-0.00774666114837,0.0127324931914,-0.0605032370936))],
        [25, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.999769709131,-0.00683690807754,0.00565692317327,0.0195393093955),PyKDL.Vector(0.143402769587,0.00560941008048,-0.0682080677974))],
        [26, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.999702001968,0.00436508873022,0.00893993421014,0.0222919455689),PyKDL.Vector(0.2867315755,0.0037977729025,-0.0723254241133))],
        [27, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.718926115108,0.0025958563067,0.000863904789675,0.695081114845),PyKDL.Vector(0.00685389266037,0.041611313921,-0.0242848250842))],
        [28, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.723920159064,-0.00406580031329,-0.00237155614562,0.689867703469),PyKDL.Vector(0.152973875805,0.0480443334089,-0.0203619760073))],
        [29, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.730592084981,-0.0115053876764,-0.00159217841913,0.682715384612),PyKDL.Vector(0.296627722109,0.0526564873934,-0.0157362559562))],
        [30, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.0107101025933,-0.707578018883,-0.00676540180519,0.706521670039),PyKDL.Vector(-0.0316984701649,0.00141765295049,-0.0308603633287))],
        [31, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.00385143207656,0.706841586598,0.00284731518612,0.707355660699),PyKDL.Vector(0.319944660728,-0.00029327409029,-0.0292236368986))],
        ]

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

        self.objects = [obj_table, obj_box, obj_grasp, obj_wall_behind, obj_wall_right, obj_ceiling]

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

        self.allowUpdateObjects()
        # start thread for updating objects' positions in openrave
        thread.start_new_thread(self.poseUpdaterThread, (None,1))

        if simulation_only:
            # create the robot interface for simulation
            velma = VelmaSim(self.openrave, velma_ikr)
        else:
            # create the robot interface for real hardware
            velma = Velma()

        self.openrave.addRobotInterface(velma)

        velma.updateTransformations()

        camera_fov_x = 50.0/180.0*math.pi
        camera_fov_y = 40.0/180.0*math.pi

#        self.openrave.setCamera(velma.T_B_C)

#        self.openrave.addCamera("head_camera",camera_fov_x, camera_fov_y, 0.2)
#        self.openrave.updatePose("head_camera", velma.T_B_C)

        k_pregrasp = Wrench(Vector3(1000.0, 1000.0, 1000.0), Vector3(300.0, 300.0, 300.0))
        k_grasp = Wrench(Vector3(800.0, 800.0, 100.0), Vector3(150.0, 150.0, 150.0))

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

        ################
        # the main loop
        ################
        base_qar = copy.deepcopy(velma.qar)
        T_B_O_prev = None
        checked_grasps_idx = []
        grips_db = []
        if True:
            self.allowUpdateObjects()
            rospy.sleep(1.0)

            T_B_M = self.getMarkerPose(25)
            T_B_Ed = T_B_M * PyKDL.Frame(PyKDL.Rotation.RotX(180.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Vector(0,0,-0.2))

            traj = self.openrave.planMoveForRightArm(T_B_Ed, None)
            if traj == None:
                print "FATAL ERROR: colud not plan trajectory"
                exit(0)

            duration = math.fsum(traj[3])

            raw_input("Press Enter to visualize the trajectory...")
            if velma.checkStopCondition():
                exit(0)
            self.openrave.showTrajectory(duration * time_mult * 0.5, qar_list=traj[4])

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
                return

            raw_input("Press Enter to set lower stiffness in 3s...")
            velma.moveImpedance(k_grasp, 3.0)
            if velma.checkStopCondition(3.0):
                exit(0)

            # move down
            velma.updateTransformations()
            T_B_Ed2 = T_B_Ed * PyKDL.Frame(PyKDL.Vector(0,0,0.25))
            T_B_Wd = T_B_Ed2 * velma.T_E_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.05, max_v_r=0.1)
            raw_input("Press Enter to move the robot in " + str(duration) + " s...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                return

            # move up
            velma.updateTransformations()
            T_B_Wd = T_B_Ed * velma.T_E_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
            raw_input("Press Enter to move the robot in " + str(duration) + " s...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                return


            raw_input("Press Enter to set bigger stiffness in 3s...")
            velma.moveImpedance(k_pregrasp, 3.0)
            if velma.checkStopCondition(3.0):
                exit(0)





            # move to base pose
            print "moving to base pose..."
            traj = self.openrave.planMoveForRightArm(None, base_qar)
            if traj == None:
                print "FATAL ERROR: colud not plan trajectory to base pose"
                return
            duration = math.fsum(traj[3])
            raw_input("Press Enter to visualize the trajectory...")
            if velma.checkStopCondition():
                exit(0)
            self.openrave.showTrajectory(duration * time_mult * 0.5, qar_list=traj[4])

            self.switchToJoint(velma)

            print "trajectory len: %s"%(len(traj[0]))
            raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(duration * time_mult + 1.0):
                exit(0)

            raw_input("Press Enter to exit...")
            exit(0)


if __name__ == '__main__':

    rospy.init_node('grasp_leanring')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


