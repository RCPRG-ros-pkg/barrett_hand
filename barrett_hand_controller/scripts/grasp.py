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

    def getMarkerPoseFake(self, marker_id, wait = True, timeBack = None):
        T_B_Tm = PyKDL.Frame( PyKDL.Vector(1.0,-0.3,0.4) )
        T_Tm_Bm = PyKDL.Frame( PyKDL.Vector(-0.06, 0.3, 0.135) )
        T_Bm_Gm = PyKDL.Frame( PyKDL.Rotation.RotZ(-30.0/180.*math.pi), PyKDL.Vector(0.1,-0.1,0.06) )
        if marker_id == 6:
            return T_B_Tm
        elif marker_id == 7:
            return T_B_Tm * T_Tm_Bm
        elif marker_id == 19:
            return T_B_Tm * T_Tm_Bm * T_Bm_Gm
        return None

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

    def spin(self):
        m_id = 0

        # create objects definitions
        obj_table = grip.GraspableObject("table", "box", [0.60,0.85,0.07])
        obj_table.addMarker( 6, PyKDL.Frame(PyKDL.Vector(0, -0.225, 0.035)) )

        obj_box = grip.GraspableObject("box", "box", [0.22,0.24,0.135])
        obj_box.addMarker( 7, PyKDL.Frame(PyKDL.Vector(-0.07, 0.085, 0.065)) )

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

        self.objects = [obj_table, obj_box, obj_grasp]

        if False:
            grip.gripUnitTest(obj_grasp)
            exit(0)

        # load and init ik solver for right hand
        velma_ikr = velmautils.VelmaIkSolver()
        velma_ikr.initIkSolver()

        simulation_only = False
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

        k_pregrasp = Wrench(Vector3(1000.0, 1000.0, 1000.0), Vector3(300.0, 300.0, 300.0))
        k_grasp = Wrench(Vector3(500.0, 500.0, 500.0), Vector3(150.0, 150.0, 150.0))

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
            velma.moveImpedance(k_pregrasp, 3.0)
            if velma.checkStopCondition(3.0):
                exit(0)

        velma.updateTransformations()
        velma_init_T_B_W = copy.deepcopy(velma.T_B_W)

        grips_db = []
        while True:
            self.disallowUpdateObjects()

            grasps,indices = self.openrave.generateGrasps("object")

            min_cost = 10000.0
            min_i = 0
            for i in range(0, len(grasps)):
                T_Br_E = self.openrave.getGraspTransform(grasps[i], collisionfree=True)
                velma.updateTransformations()
                traj_T_B_Ed = [velma.T_B_W * velma.T_W_E, T_Br_E]
                cost = velma_ikr.getTrajCost(traj_T_B_Ed, velma.qar, velma.T_B_T2.Inverse(), False, False)
                print "%s   cost: %s"%(i,cost)
                if cost < min_cost:
                    min_cost = cost
                    min_i = i

            if min_cost > 1000.0:
                print "could not reach the destination point"
                break

            print "found grasp"
            grasp = grasps[min_i]

            T_Br_E = self.openrave.getGraspTransform(grasp, collisionfree=True)
            self.openrave.showGrasp(grasp)

            T_B_Wd = T_Br_E * velma.T_E_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
            velma.moveWrist2(T_B_Wd*velma.T_W_T)

            qar_list = []
            for f in np.linspace(0.0, 1.0, 50):
                q_out, T_B_E = velma_ikr.simulateTrajectory(velma.T_B_W * velma.T_W_E, T_B_Wd * velma.T_W_E, f, velma.qar, velma.T_B_T2.Inverse())
                qar_list.append(q_out)
            self.openrave.showTrajectory(0.1, qar_list=qar_list)

            final_config = self.openrave.getFinalConfig(grasp)
            print "final_config:"
            print final_config
            print "standoff: %s"%(self.openrave.getGraspStandoff(grasp))

            raw_input("Press Enter to move the robot in " + str(duration) + " s...")
            if velma.checkStopCondition():
                break
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

            raw_input("Press Enter to close fingers for pre-grasp...")
            # close the fingers for pre-grasp
            ad = 10.0/180.0*math.pi
            velma.move_hand_client([final_config[0]-ad, final_config[1]-ad, final_config[2]-ad, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))

            print "setting stiffness to lower value"
            velma.moveImpedance(k_grasp, 3.0)
            if velma.checkStopCondition(3.0):
                break

#            print "ok"
#            while not rospy.is_shutdown():
#                rospy.sleep(0.5)
#            exit(0)


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


