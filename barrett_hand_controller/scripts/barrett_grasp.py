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
from velma import Velma
from velmasim import VelmaSim
import random
from openravepy import *
from openravepy.openravepy_int import KinBody, TriMesh

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import velmautils
import surfaceutils
import openraveinstance
import itertools
import dijkstra
import grip
import operator

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

import ode
import xode.transform
import volumetricutils

from multiprocessing import Process, Queue

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler


def contactToWrenches(pos, normal, friction, Nconepoints):
            wrenches = []
            fdeltaang = 2.0*math.pi/float(Nconepoints)
            nz = normal
            if abs(nz.z()) < 0.7:
                nx = PyKDL.Vector(0,0,1)
            elif abs(nz.y()) < 0.7:
                nx = PyKDL.Vector(0,1,0)
            else:
                nx = PyKDL.Vector(1,0,0)
            ny = nz * nx
            nx = ny * nz
            nx.Normalize()
            ny.Normalize()
            nz.Normalize()
            R_n = PyKDL.Frame(PyKDL.Rotation(nx,ny,nz))
            fangle = 0.0
            for cp in range(Nconepoints):
                nn = R_n * PyKDL.Frame(PyKDL.Rotation.RotZ(fangle)) * PyKDL.Vector(friction,0,1)
                fangle += fdeltaang
                tr = pos * nn
                wr = PyKDL.Wrench(nn,tr)
                wrenches.append([wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]])
            return wrenches

def getGraspQHull(points, openrave_grasper):
    try:
        planes, faces, triangles = openrave_grasper.ConvexHull(np.array(points), returnplanes=True,returnfaces=False,returntriangles=False)
    except AttributeError as e1:
        print e1
        return None
    return planes

def KDLToOpenrave(T):
        ret = numpy.array([
        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
        [0, 0, 0, 1]])
        return ret

def OpenraveToKDL(T):
        rot = PyKDL.Rotation(T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2])
        pos = PyKDL.Vector(T[0][3], T[1][3], T[2][3])
        return PyKDL.Frame(rot, pos)

class GripperLinkGeometricModel:
    def __init__(self, name, openrave_robot, openrave_grasper):
        link = openrave_robot.GetLink(name)
        col = link.GetCollisionData()
        vertices = col.vertices
        faces = col.indices

        print "sampling the surface..."
        self.surface_points = surfaceutils.sampleMeshDetailedRays(vertices, faces, 0.0015)
        print "surface has %s points"%(len(self.surface_points))

        surface_points_init = []
        for sp in self.surface_points:
            surface_points_init.append(sp)

        p_idx = random.randint(0, len(self.surface_points)-1)
        p_dist = 0.004

        print "generating a subset of surface points..."

        self.sampled_points = []
        while True:
            self.sampled_points.append(p_idx)
            surface_points2 = []
            for sp in surface_points_init:
                if (sp.pos-self.surface_points[p_idx].pos).Norm() > p_dist:
                    surface_points2.append(sp)
            if len(surface_points2) == 0:
                break
            surface_points_init = surface_points2
            p_idx = surface_points_init[0].id

        print "subset size: %s"%(len(self.sampled_points))

        points_for_qhull = []
        for pt_idx in self.sampled_points:
            pos = self.surface_points[pt_idx].pos
            points_for_qhull.append([pos[0], pos[1], pos[2]])
        print "generating qhull for link"
        self.qhull_planes = getGraspQHull(points_for_qhull, openrave_grasper)
        print "link qhull planes: %s"%(len(self.qhull_planes))


#        T_W_O = PyKDL.Frame(PyKDL.Vector(0,0,0.5))
        for pt_idx in self.sampled_points:
                indices, nx, pc1, pc2 = surfaceutils.pclPrincipalCurvaturesEstimation(self.surface_points, pt_idx, 5, 0.003)
#                m_id = self.pub_marker.publishVectorMarker(T_W_O * surface_points[pt_idx].pos, T_W_O * (surface_points[pt_idx].pos + nx*0.004), m_id, 1, 0, 0, frame='world', namespace='default', scale=0.0002)
#                m_id = self.pub_marker.publishVectorMarker(T_W_O * surface_points[pt_idx].pos, T_W_O * (surface_points[pt_idx].pos + surface_points[pt_idx].normal*0.004), m_id, 0, 0, 1, frame='world', namespace='default', scale=0.0002)

                self.surface_points[pt_idx].frame = PyKDL.Frame(PyKDL.Rotation(nx, self.surface_points[pt_idx].normal * nx, self.surface_points[pt_idx].normal), self.surface_points[pt_idx].pos)
                self.surface_points[pt_idx].pc1 = pc1
                self.surface_points[pt_idx].pc2 = pc2
                if pc1 < 0.2:
                    self.surface_points[pt_idx].is_plane = True
                    self.surface_points[pt_idx].is_edge = False
                    self.surface_points[pt_idx].is_point = False
                elif pc2 < 0.2:
                    self.surface_points[pt_idx].is_plane = False
                    self.surface_points[pt_idx].is_edge = True
                    self.surface_points[pt_idx].is_point = False
                else:
                    self.surface_points[pt_idx].is_plane = False
                    self.surface_points[pt_idx].is_edge = False
                    self.surface_points[pt_idx].is_point = True

        print "generating a set of interior points..."
        margin = 0.002
#            vertices_min, vertices_max = velmautils.getMeshBB(vertices, faces)

        self.points_in_link = []
        for pt_idx in self.sampled_points:
                if self.surface_points[pt_idx].pc1 > 0.2:
                    continue
                self.points_in_link.append(self.surface_points[pt_idx].pos - self.surface_points[pt_idx].normal*margin)

        print "points_in_link: %s"%(len(self.points_in_link))

class GripperLinkModel:
    def __init__(self, name, link_geom):
        self.name = name
        self.link_geom = link_geom
        self.intersecting_links = []

class GripperModel:
    def __init__(self, openrave_robot, openrave_grasper):
        self.openrave_robot = openrave_robot
        link_three_geom = GripperLinkGeometricModel("right_HandFingerOneKnuckleThreeLink", openrave_robot, openrave_grasper)
        link_two_geom = GripperLinkGeometricModel("right_HandFingerOneKnuckleTwoLink", openrave_robot, openrave_grasper)

        self.links = {}
        self.links["right_HandFingerOneKnuckleTwoLink"] = GripperLinkModel("right_HandFingerOneKnuckleTwoLink", link_two_geom)
        self.links["right_HandFingerOneKnuckleThreeLink"] = GripperLinkModel("right_HandFingerOneKnuckleThreeLink", link_three_geom)
        self.links["right_HandFingerTwoKnuckleTwoLink"] = GripperLinkModel("right_HandFingerTwoKnuckleTwoLink", link_two_geom)
        self.links["right_HandFingerTwoKnuckleThreeLink"] = GripperLinkModel("right_HandFingerTwoKnuckleThreeLink", link_three_geom)
        self.links["right_HandFingerThreeKnuckleTwoLink"] = GripperLinkModel("right_HandFingerThreeKnuckleTwoLink", link_two_geom)
        self.links["right_HandFingerThreeKnuckleThreeLink"] = GripperLinkModel("right_HandFingerThreeKnuckleThreeLink", link_three_geom)

        self.links["right_HandFingerOneKnuckleTwoLink"].intersecting_links.append("right_HandFingerOneKnuckleThreeLink")
        self.links["right_HandFingerOneKnuckleThreeLink"].intersecting_links.append("right_HandFingerOneKnuckleTwoLink")
        self.links["right_HandFingerTwoKnuckleTwoLink"].intersecting_links.append("right_HandFingerTwoKnuckleThreeLink")
        self.links["right_HandFingerTwoKnuckleThreeLink"].intersecting_links.append("right_HandFingerTwoKnuckleTwoLink")
        self.links["right_HandFingerThreeKnuckleTwoLink"].intersecting_links.append("right_HandFingerThreeKnuckleThreeLink")
        self.links["right_HandFingerThreeKnuckleThreeLink"].intersecting_links.append("right_HandFingerThreeKnuckleTwoLink")

        #
        # discretize the configuration of the gripper
        #
        self.sp_configs = []
        self.f1_configs = []
        self.f2_configs = []
        self.f3_configs = []
        for sp in np.linspace(0.01, 179.99, 12):
            self.sp_configs.append(sp)
#        for fx in np.linspace(80.0, 130.0, 20):
        for fx in np.linspace(80.0, 130.0, 40):
            self.f1_configs.append(fx)
            self.f2_configs.append(fx)
            self.f3_configs.append(fx)

        self.dof_configs = (self.sp_configs, self.f1_configs, self.f3_configs, self.f2_configs)
        self.dof_limits = self.openrave_robot.GetDOFLimits()

    def getAnglesForConfigIdx(self, cf_idx):
        return np.array( [ self.sp_configs[cf_idx[0]], self.f1_configs[cf_idx[1]], self.f3_configs[cf_idx[2]], self.f2_configs[cf_idx[3]] ] )

    def getSurfacePointsForConfig(self, cf, T_E_W):


        config_link_map = {}
        config_link_map[(0,1)] = [0, ["right_HandFingerOneKnuckleThreeLink", "right_HandFingerOneKnuckleTwoLink"]]
        config_link_map[(0,3)] = [1, ["right_HandFingerTwoKnuckleThreeLink", "right_HandFingerTwoKnuckleTwoLink"]]
        config_link_map[(2,)] = [2, [ "right_HandFingerThreeKnuckleThreeLink", "right_HandFingerThreeKnuckleTwoLink"]]

        non_actuated_joints = (0,)

        points = []
        points_forbidden = []

        cf_values = [0,0,0,0]
        cf_values_add = [0,0,0,0]
        valid_values = None
        for cf_idx in range(len(cf)):
            if cf[cf_idx] != None:
                if valid_values == None:
                    valid_values = (cf_idx,)
                else:
                    valid_values = valid_values + (cf_idx,)
                cf_values[cf_idx] = self.dof_configs[cf_idx][cf[cf_idx]]/180.0*math.pi
                if cf_idx in non_actuated_joints:
                    cf_values_add[cf_idx] = cf_values[cf_idx]
                else:
                    cf_values_add[cf_idx] = cf_values[cf_idx] + 0.01
                    if cf_values_add[cf_idx] > self.dof_limits[1][cf_idx]:
                        cf_values_add[cf_idx] = cf_values[cf_idx]

        T_E_L = {}
        T_L_E = {}
        TR_E_L ={}
        self.openrave_robot.SetDOFValues(cf_values)
        for link_name in config_link_map[valid_values][1]:
            link = self.openrave_robot.GetLink(link_name)
            T_E_L[link_name] = T_E_W * OpenraveToKDL(link.GetTransform())
            T_L_E[link_name] = T_E_L[link_name].Inverse()
            TR_E_L[link_name] = PyKDL.Frame(T_E_L[link_name].M)

        T_E_L_next = {}
        self.openrave_robot.SetDOFValues(cf_values_add)
        for link_name in config_link_map[valid_values][1]:
            link = self.openrave_robot.GetLink(link_name)
            T_E_L_next[link_name] = T_E_W * OpenraveToKDL(link.GetTransform())

        for link_name in config_link_map[valid_values][1]:
            for pt_idx in self.links[link_name].link_geom.sampled_points:
                pt_L = self.links[link_name].link_geom.surface_points[pt_idx]
                pt_E = T_E_L[link_name] * pt_L.pos
                pt_remove = True
                for int_name in self.links[link_name].intersecting_links:
                    pt_L_int = T_L_E[int_name] * pt_E
                    for pl in self.links[int_name].link_geom.qhull_planes:
                        if pt_L_int[0] * pl[0] + pt_L_int[1] * pl[1] + pt_L_int[2] * pl[2] + pl[3] > 0:
                             pt_remove = False
                             break
                    if pt_remove:
                        break
                if pt_remove:
                    continue
                pt_E_next = T_E_L_next[link_name] * pt_L.pos
                v_E = pt_E_next - pt_E
                v_E.Normalize()
                n_E = TR_E_L[link_name] * pt_L.normal
                if PyKDL.dot(v_E, n_E) > 0.5:
                    direction = 1
                elif PyKDL.dot(v_E, n_E) < -0.5:
                    direction = -1
                else:
                    direction = 0

                if pt_L.is_plane == True:
                    surf_type = 0
                elif pt_L.is_edge == True:
                    surf_type = 1
                elif pt_L.is_point == True:
                    surf_type = 2
                else:
                    print "ERROR: getSurfacePointsForConfig: unknown surface type"
                points.append( (config_link_map[valid_values][0], pt_E, n_E, surf_type, cf, direction) )
            for pt_L in self.links[link_name].link_geom.points_in_link:
                pt_E = T_E_L[link_name] * pt_L
                points_forbidden.append( (config_link_map[valid_values][0], pt_E, None, None, cf, None) )
        return points, points_forbidden

    def generateSamples(self):
        points = []
        points_forbidden = []

        T_W_E = OpenraveToKDL(self.openrave_robot.GetLink("right_HandPalmLink").GetTransform())
        TR_W_E = PyKDL.Frame(T_W_E.M)
        T_E_W = T_W_E.Inverse()

        configs = []
        for sp_idx in range(len(self.sp_configs)):
            for f1_idx in range(len(self.f1_configs)):
                configs.append((sp_idx, f1_idx, None, None))
            for f2_idx in range(len(self.f2_configs)):
                configs.append((sp_idx, None, None, f2_idx))
        for f3_idx in range(len(self.f3_configs)):
            configs.append((None, None, f3_idx, None))

        print "generateSamples: configs: %s"%(len(configs))
        for cf in configs:
            p, pf = self.getSurfacePointsForConfig(cf, T_E_W)
            points += p
            points_forbidden += pf
        return points, points_forbidden

    def generateSelfCollisionData(self):
        self_collisions_configs = set()
        for sp_idx in range(len(self.sp_configs)):
            print "%s / %s"%(sp_idx, len(self.sp_configs))
            sp = self.sp_configs[sp_idx]
            for f1_idx in range(len(self.f1_configs)):
                f1 = self.f1_configs[f1_idx]
                for f2_idx in range(len(self.f2_configs)):
                    f2 = self.f2_configs[f2_idx]
                    for f3_idx in range(len(self.f3_configs)):
                        f3 = self.f3_configs[f3_idx]
                        self.openrave_robot.SetDOFValues([sp/180.0*math.pi, f1/180.0*math.pi, f3/180.0*math.pi, f2/180.0*math.pi])
                        if self.openrave_robot.CheckSelfCollision():
                            self_collisions_configs.add( (sp_idx, f1_idx, f3_idx, f2_idx) )
        return self_collisions_configs

class VolumetricGrasp:
    def __init__(self):
        self.hand_config = None
        self.obj_ori_idx = None
        self.contacts_obj = None
        self.contacts_link = None
        self.normals_obj = None
        self.contacts_obj_reduced = None
        self.wrenches = None
        self.obj_pos_E = None
        self.normals_link = None
        self.dof_directions = None
        self.contacts_link_reduced = None
        self.wrenches_link = None

    def calculateWrenches(self):
        contacts_cf_ori = {}
        wrenches_cf_ori = {}
        self.contacts_obj_reduced = []
        self.wrenches = []
        for cf_idx in range(len(self.contacts_obj)):
                contacts = []
                for pt_idx in range(len(self.contacts_obj[cf_idx])):
                    pt_E = self.contacts_obj[cf_idx][pt_idx]
                    norm_E = self.normals_obj[cf_idx][pt_idx]
                    similar = False
                    for c in contacts:
                        if (c[0]-pt_E).Norm() < 0.003 and velmautils.getAngle(c[1],norm_E) < 20.0/180.0*math.pi:
                            similar = True
                            break
                    if not similar:
                        contacts.append((pt_E, norm_E))
                self.contacts_obj_reduced.append(contacts)
                self.wrenches.append([])
                for c in contacts:
                    self.wrenches[-1] += contactToWrenches(c[0]-self.obj_pos_E, c[1], 0, 1)

    def calculateWrenchesOnLink(self):
        contacts_cf_ori = {}
        wrenches_cf_ori = {}
        self.contacts_link_reduced = []
        self.wrenches_link = []
        for cf_idx in range(len(self.contacts_link)):
                contacts = []
                for pt_idx in range(len(self.contacts_link[cf_idx])):
                    pt_E = self.contacts_link[cf_idx][pt_idx]
                    norm_E = self.normals_link[cf_idx][pt_idx]
                    similar = False
                    for c in contacts:
                        if (c[0]-pt_E).Norm() < 0.003 and velmautils.getAngle(c[1],norm_E) < 20.0/180.0*math.pi:
                            similar = True
                            break
                    if not similar:
                        contacts.append((pt_E, norm_E))
                self.contacts_link_reduced.append(contacts)
                self.wrenches_link.append([])
                for c in contacts:
                    self.wrenches_link[-1] += contactToWrenches(c[0]-self.obj_pos_E, c[1], 0, 1)

    def saveToFile(self, file_instance):
        def writePoints(points_tuple, file_instance):
            for points in points_tuple:
                for pt in points:
                    file_instance.write( str(pt[0]) + " " + str(pt[1]) + " " + str(pt[2]) + " ")
                file_instance.write("\n")

        file_instance.write( str(self.hand_config[0]) + " " + str(self.hand_config[1]) + " " + str(self.hand_config[2]) + " " + str(self.hand_config[3]) +
        " " + str(self.obj_ori_idx) + " " + str(self.obj_pos_E[0]) + " " + str(self.obj_pos_E[1]) + " " + str(self.obj_pos_E[2]) + 
        " " + str(self.dof_directions[0]) + " " + str(self.dof_directions[1]) + " " + str(self.dof_directions[2]) + "\n")

        writePoints(self.contacts_obj, file_instance)
        writePoints(self.contacts_link, file_instance)
        writePoints(self.normals_obj, file_instance)
        writePoints(self.normals_link, file_instance)

    def loadFromFile(self, file_instance):
        def readPoints(file_instance):
            points = []
            for cf_idx in range(3):
                points.append([])
                line = file_instance.readline().split()
                for i in range(0, len(line), 3):
                    points[-1].append( PyKDL.Vector(float(line[i+0]), float(line[i+1]), float(line[i+2])) )
            return (points[0], points[1], points[2])

        line = file_instance.readline().split()
        if len(line) < 11:
            return False

        self.hand_config = (int(line[0]), int(line[1]), int(line[2]), int(line[3]))
        self.obj_ori_idx = int(line[4])
        self.obj_pos_E = PyKDL.Vector(float(line[5]), float(line[6]), float(line[7]))
        self.dof_directions = (int(line[8]), int(line[9]), int(line[10]))

        self.contacts_obj = readPoints(file_instance)
        self.contacts_link = readPoints(file_instance)
        self.normals_obj = readPoints(file_instance)
        self.normals_link = readPoints(file_instance)

        return True

    def loadFromFileFast(self, file_instance):
        line = file_instance.readline().split()
        if len(line) < 11:
            return False

        self.hand_config = (int(line[0]), int(line[1]), int(line[2]), int(line[3]))
        self.obj_ori_idx = int(line[4])
        self.obj_pos_E = PyKDL.Vector(float(line[5]), float(line[6]), float(line[7]))
        self.dof_directions = (int(line[8]), int(line[9]), int(line[10]))

        for i in range(12):
            file_instance.readline()

        return True

class GraspingTask:
    """
Class for grasp learning.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = pub_marker
        self.listener = tf.TransformListener();
        # create an interactive marker server on the topic namespace simple_marker
        self.mk_server = InteractiveMarkerServer('obj_pose_markers')

    def addBox(self, name, x_size, y_size, z_size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromBoxes(numpy.array([[0,0,0,0.5*x_size,0.5*y_size,0.5*z_size]]),True)
        self.env.Add(body,True)

    def addSphere(self, name, size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromSpheres(numpy.array([[0,0,0,0.5*size]]),True)
        self.env.Add(body,True)

    def addTrimesh(self, name, vertices, faces):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        mesh = TriMesh()
        mesh.vertices = copy.deepcopy(vertices)
        mesh.indices = copy.deepcopy(faces)
        body.InitFromTrimesh(mesh, True)
        self.env.Add(body,True)

    def updatePose(self, name, T_Br_Bo):
        with self.env:
            body = self.env.GetKinBody(name)
            if body != None:
                body.SetTransform(KDLToOpenrave(T_Br_Bo))
            else:
#                print "openrave: could not find body: %s"%(name)
                pass
            self.env.UpdatePublishedBodies()

    # spread, f1, f3, f2
    def runGrasp(self, directions, positions):
        contacts = None
        finalconfig = None
        mindist = None
        volume = None
        with self.openrave_robot.CreateRobotStateSaver():
            try:
                    lower_limit, upper_limit = self.openrave_robot.GetDOFLimits()
                    pos = []
                    for i in range(4):
                        if directions[i] > 0.0:
                            pos.append(lower_limit[i]+0.001)
                        elif directions[i] < 0.0:
                            pos.append(upper_limit[i]-0.051)
                        else:
                            pos.append(positions[i])

                    self.openrave_robot.GetActiveManipulator().SetChuckingDirection(directions)
                    target = self.env.GetKinBody("object")
                    self.openrave_robot.SetDOFValues(pos)

                    contacts,finalconfig,mindist,volume = self.grasper.Grasp(execute=False, outputfinal=True, transformrobot=False, target=target)
                    
            except e:
                print "runGrasp: planning error:"
                print e
        return contacts,finalconfig,mindist,volume

    def generateGWS(self, contacts, friction):
        qhullpoints = []
        qhullpoints_contact_idx = []
        contact_idx = 0
        for c in contacts:
            p = PyKDL.Vector(c[0], c[1], c[2])
            nz = PyKDL.Vector(c[3], c[4], c[5])
            wrs = contactToWrenches(p, nz, friction, 6)
            for wr in wrs:
                qhullpoints.append([wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]])
                qhullpoints_contact_idx.append(contact_idx)
            contact_idx += 1

        qhullplanes = getGraspQHull(qhullpoints, self.grasper)
        qhullplanes_contacts = []
        contact_planes = []
        for pl in qhullplanes:
            qhullplanes_contacts.append([])

        for c in contacts:
            contact_planes.append([])

        for pt_idx in range(len(qhullpoints)):
            pt = qhullpoints[pt_idx]
            contact_idx = qhullpoints_contact_idx[pt_idx]

            for pl_idx in range(len(qhullplanes)):
                pl = qhullplanes[pl_idx]
                dist = pl[0] * pt[0] + pl[1] * pt[1] + pl[2] * pt[2] + pl[3] * pt[3] + pl[4] * pt[4] + pl[5] * pt[5] + pl[6]
                if abs(dist) < 0.00000001 and not contact_idx in qhullplanes_contacts[pl_idx]:
                    qhullplanes_contacts[pl_idx].append(contact_idx)
                    contact_planes[contact_idx].append(pl_idx)

        return qhullplanes, contact_planes

    def reduceContacts(self, contacts):
        max_angle = 15.0/180*math.pi
        max_n_dist = 2.0 * math.sin(max_angle/2.0)
        max_pos_dist = 0.003
        removed_ids = []
        reduced_contacts = []
        for c1_id in range(len(contacts)):
            if c1_id in removed_ids:
                continue
            removed_ids.append(c1_id)
            c1 = contacts[c1_id]
            reduced_contacts.append(c1)
            c1_pos = PyKDL.Vector(c1[0], c1[1], c1[2])
            c1_n = PyKDL.Vector(c1[3], c1[4], c1[5])
            for c2_id in range(c1_id+1, len(contacts)):
                if c2_id in removed_ids:
                    continue
                c2 = contacts[c2_id]
                c2_pos = PyKDL.Vector(c2[0], c2[1], c2[2])
                c2_n = PyKDL.Vector(c2[3], c2[4], c2[5])
                if (c1_pos-c2_pos).Norm() < max_pos_dist and (c1_n-c2_n).Norm() < max_n_dist:
                    removed_ids.append(c2_id)

        return reduced_contacts                    

        dist = np.zeros((len(contacts),len(contacts)))
        for c1_id in range(len(contacts)):
            c1 = contacts[c1_id]
            c1_pos = PyKDL.Vector(c1[0], c1[1], c1[2])
            c1_n = PyKDL.Vector(c1[3], c1[4], c1[5])
            for c2_id in range(0, c1_id):
                c2 = contacts[c2_id]
                c2_pos = PyKDL.Vector(c2[0], c2[1], c2[2])
                c2_n = PyKDL.Vector(c2[3], c2[4], c2[5])

                dist[c1_id][c2_id] = (c1_pos-c2_pos).Norm()/max_pos_dist + (c1_n-c2_n).Norm()/max_n_dist

    def getMesh(self, name):
        body = self.env.GetKinBody(name)
        if body == None:
            return None
        link = body.GetLinks()[0]
        col = link.GetCollisionData()
        return col.vertices, col.indices

    def checkNeighbors(self, surface_points, current_id, ref_force, ref_torque, max_force_dist, max_torque_dist):
        force = surface_points[current_id].normal
        if (ref_force-force).Norm() > max_force_dist:
            return []

        torque = surface_points[current_id].pos * surface_points[current_id].normal
        if (ref_torque-torque).Norm() > max_torque_dist:
            return []

        if surface_points[current_id].visited == True:
            return []
        ret_list = [current_id]
        surface_points[current_id].visited = True
        for n_id in surface_points[current_id].neighbors_id:
            ret_list = ret_list + self.checkNeighbors(surface_points, n_id, ref_force, ref_torque, max_force_dist, max_torque_dist)

        return ret_list

    def getContactPointOnSurface(self, surface_points, contact_point):
        min_dist = None
        min_point_id = -1
        for pt in surface_points:
            pt.visited = False
            dist = (pt.pos-contact_point).Norm()
            if min_dist == None or dist < min_dist:
                min_dist = dist
                min_point_id = pt.id
        return min_point_id

    def getContactRegion(self, surface_points, contact_point):
        min_dist = None
        min_point_id = -1
        for pt in surface_points:
            pt.visited = False
            dist = (pt.pos-contact_point).Norm()
            if min_dist == None or dist < min_dist:
                min_dist = dist
                min_point_id = pt.id

        max_force_dist = 0.1
        max_torque_dist = 0.02

        ref_force = surface_points[min_point_id].normal
        ref_torque = surface_points[min_point_id].pos * surface_points[min_point_id].normal

        region = self.checkNeighbors(surface_points, min_point_id, ref_force, ref_torque, max_force_dist, max_torque_dist)
        return region

    def getContactConstraintFn(self, surface_points, current_id, ref_force, ref_torque, max_force_dist, max_torque_dist, contact_point):

        force = surface_points[current_id].normal
        torque = surface_points[current_id].pos * surface_points[current_id].normal
        if (ref_force-force).Norm() > max_force_dist or (ref_torque-torque).Norm() > max_torque_dist:
            dist = (contact_point-surface_points[current_id].pos).Norm()
            if self.getContactConstraint_min_dist == None or self.getContactConstraint_min_dist > dist:
                self.getContactConstraint_min_dist = dist
            return

        if surface_points[current_id].visited == True:
            return

        surface_points[current_id].visited = True
        for n_id in surface_points[current_id].neighbors_id:
            self.getContactConstraintFn(surface_points, n_id, ref_force, ref_torque, max_force_dist, max_torque_dist, contact_point)

        return

    def getContactConstraint(self, surface_points, point_id):
        for pt in surface_points:
            pt.visited = False

        self.getContactConstraint_min_dist = None

        max_force_dist = 0.2
        max_torque_dist = 0.02

        ref_force = surface_points[point_id].normal
        ref_torque = surface_points[point_id].pos * surface_points[point_id].normal

        self.getContactConstraintFn(surface_points, point_id, ref_force, ref_torque, max_force_dist, max_torque_dist, surface_points[point_id].pos)
        return self.getContactConstraint_min_dist

    def getQualituMeasure(self, qhull):
        if qhull == None:
            return 0.0

        mindist = None
        for qp in qhull:
            if qp[6] >= 0:
                return 0
            if mindist == None or mindist > -qp[6]:
                mindist = -qp[6]

        return mindist

    def getQualituMeasure2(self, qhull, wr):
        if qhull == None:
            return 0.0

        wr6 = [wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]]
        mindist = None
        for qp in qhull:
            n = np.array([qp[0],qp[1],qp[2],qp[3],qp[4],qp[5]])
            if np.dot(n,n) > 1.00001 or np.dot(n,n) < 0.9999:
                print "ERROR: getQualituMeasure2: np.dot(n,n): %s"%(np.dot(n,n))
                exit(0)
            dot = np.dot(np.array(wr6), n)
            if dot > 0:
                dqp = -qp[6]/dot
                if mindist == None or mindist > dqp:
                    mindist = dqp
        return mindist

    def saveSelfCollisionData(self, filename, self_collisions_configs):
        with open(filename, 'w') as f:
            for cf in self_collisions_configs:
                f.write(str(cf[0]) + " " + str(cf[1]) + " " + str(cf[2]) + " " + str(cf[3]) + "\n")

    def loadSelfCollisionData(self, filename):
        self_collisions_configs = set()
        with open(filename, 'r') as f:
            while True:
                cf_str = f.readline().split()
                if len(cf_str) != 4:
                    break
                self_collisions_configs.add( (int(cf_str[0]), int(cf_str[1]), int(cf_str[2]), int(cf_str[3])) )
        return self_collisions_configs

    def visualize1(self, T_W_H, joint_map, points_for_config, ori_for_config, vol_obj, sp_configs, f1_configs, f2_configs, f3_configs, pub_marker):
                  for cf in points_for_config:
                      # update the gripper visualization in ros
                      cf2 = [0,0,0,0]
                      cf2[0] = cf[0] if cf[0] != None else 0
                      cf2[1] = cf[1] if cf[1] != None else 0
                      cf2[2] = cf[2] if cf[2] != None else 0
                      cf2[3] = cf[3] if cf[3] != None else 0
                      self.openrave_robot.SetDOFValues([sp_configs[cf2[0]]/180.0*math.pi,f1_configs[cf2[1]]/180.0*math.pi,f3_configs[cf2[2]]/180.0*math.pi,f2_configs[cf2[3]]/180.0*math.pi])
                      for i in range(0, 2):
                          js = JointState()
                          js.header.stamp = rospy.Time.now()
                          for jn in joint_map:
                              js.name.append(joint_map[jn])
                              js.position.append(self.openrave_robot.GetJoint(jn).GetValue(0))
                          self.pub_js.publish(js)
                          rospy.sleep(0.1)

                      pub_marker.eraseMarkers(0,6000, frame_id='world')
                      print "orientations: %s"%(len(ori_for_config[cf]))
                      m_id = 0
                      for ori in ori_for_config[cf]:
                          T_W_O = T_W_H * vol_obj.orientations[ori] * vol_obj.T_H_O
                          m_id = pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)
                          rospy.sleep(0.001)

                      for ori in ori_for_config[cf]:
                          o_q = orientations[ori].M.GetQuaternion()
                          o_pt = PyKDL.Vector(o_q[0], o_q[1], o_q[2])
                          m_id = pub_marker.publishSinglePointMarker(PyKDL.Vector(0,0,0.6) + o_pt*0.1, m_id, r=0, g=o_q[3], b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                          rospy.sleep(0.001)

                      while True:
                          ch = raw_input("x")
                          if ch == 'n':
                              break
                          pub_marker.eraseMarkers(0,6000, frame_id='world')
                          ori = random.choice(tuple(ori_for_config[cf]))
                          T_W_O = T_W_H * vol_obj.orientations[ori] * vol_obj.T_H_O
                          m_id = pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=0, g=1, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)

    def visualize2(self, T_W_E, good_grasps, vol_obj, gripper_model, pub_marker, joint_map):
        for grasp in good_grasps:
            cf = grasp.hand_config
            ori = grasp.obj_ori_idx

            TT_E_H = PyKDL.Frame(grasp.obj_pos_E)
            T_W_H = T_W_E * TT_E_H

            self.openrave_robot.SetDOFValues([gripper_model.sp_configs[cf[0]]/180.0*math.pi,gripper_model.f1_configs[cf[1]]/180.0*math.pi,gripper_model.f3_configs[cf[2]]/180.0*math.pi,gripper_model.f2_configs[cf[3]]/180.0*math.pi])
            for i in range(0, 2):
                # update the gripper visualization in ros
                js = JointState()
                js.header.stamp = rospy.Time.now()
                for jn in joint_map:
                    js.name.append(joint_map[jn])
                    js.position.append(self.openrave_robot.GetJoint(jn).GetValue(0))
                self.pub_js.publish(js)
                rospy.sleep(0.1)
            print cf, ori
            pub_marker.eraseMarkers(0,2000, frame_id='world')
            m_id = 0
            T_W_O = T_W_H * vol_obj.orientations[ori] * vol_obj.T_H_O
            m_id = pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)
            for cf_idx in range(len(grasp.contacts_obj_reduced)):
                for c in grasp.contacts_obj_reduced[cf_idx]:
                    m_id = pub_marker.publishVectorMarker(T_W_E*c[0], T_W_E*(c[0]+c[1]*0.004), m_id, r=1, g=1, b=1, namespace='default', frame='world', scale=0.0005)

            for cf_idx in range(len(grasp.contacts_link)):
                for pt in grasp.contacts_link[cf_idx]:
                    if grasp.dof_directions[cf_idx] == 1:
                        color = (0,1,0)
                    elif grasp.dof_directions[cf_idx] == -1:
                        color = (1,0,0)
                    elif grasp.dof_directions[cf_idx] == 0:
                        color = (1,1,1)
                    else:
                        print "ERROR: direction: %s"%(grasp.dof_directions[cf_idx])
                        color = (0,0,0)
                    m_id = pub_marker.publishSinglePointMarker(pt, m_id, r=color[0], g=color[1], b=color[2], namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=T_W_E)

            raw_input("Press ENTER to continue...")

    def visualizeGrasp(self, T_W_E, grasp, vol_obj, gripper_model, pub_marker, joint_map):
            cf = grasp.hand_config
            ori = grasp.obj_ori_idx

            TT_E_H = PyKDL.Frame(grasp.obj_pos_E)
            T_W_H = T_W_E * TT_E_H

            self.openrave_robot.SetDOFValues([gripper_model.sp_configs[cf[0]]/180.0*math.pi,gripper_model.f1_configs[cf[1]]/180.0*math.pi,gripper_model.f3_configs[cf[2]]/180.0*math.pi,gripper_model.f2_configs[cf[3]]/180.0*math.pi])
            for i in range(0, 2):
                # update the gripper visualization in ros
                js = JointState()
                js.header.stamp = rospy.Time.now()
                for jn in joint_map:
                    js.name.append(joint_map[jn])
                    js.position.append(self.openrave_robot.GetJoint(jn).GetValue(0))
                self.pub_js.publish(js)
                rospy.sleep(0.1)
            print cf, ori
            pub_marker.eraseMarkers(0,2000, frame_id='world')
            m_id = 0
            T_W_O = T_W_H * vol_obj.orientations[ori] * vol_obj.T_H_O
            m_id = pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)
            for cf_idx in range(len(grasp.contacts_obj_reduced)):
                for c in grasp.contacts_obj_reduced[cf_idx]:
                    m_id = pub_marker.publishVectorMarker(T_W_E*c[0], T_W_E*(c[0]+c[1]*0.004), m_id, r=1, g=1, b=1, namespace='default', frame='world', scale=0.0005)

            for cf_idx in range(len(grasp.contacts_link)):
                for pt in grasp.contacts_link[cf_idx]:
                    if grasp.dof_directions[cf_idx] == 1:
                        color = (0,1,0)
                    elif grasp.dof_directions[cf_idx] == -1:
                        color = (1,0,0)
                    elif grasp.dof_directions[cf_idx] == 0:
                        color = (1,1,1)
                    else:
                        print "ERROR: direction: %s"%(grasp.dof_directions[cf_idx])
                        color = (0,0,0)
                    m_id = pub_marker.publishSinglePointMarker(pt, m_id, r=color[0], g=color[1], b=color[2], namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=T_W_E)

    def sortPointsByConfig(self, points_in_sphere, points_f_in_sphere, valid_configurations):
              # returned values
              points_for_config = {}
              points_f_for_config = {}

              # get the intersection of gripper configurations for f1 and f2 (for spread angle)
              f1_spread_values = []
              for f1_q in valid_configurations[0]:
                  if not f1_q[0] in f1_spread_values:
                      f1_spread_values.append(f1_q[0])
              spread_values = []
              for f2_q in valid_configurations[1]:
                  if f2_q[0] in f1_spread_values:
                      spread_values.append(f2_q[0])

              if len(spread_values) == 0:
                  return points_for_config, points_f_for_config

              f1_valid_configurations = []
              for f1_q in valid_configurations[0]:
                  if f1_q[0] in spread_values:
                      if not f1_q in f1_valid_configurations:
                          f1_valid_configurations.append(f1_q)
              f2_valid_configurations = []
              for f2_q in valid_configurations[1]:
                  if f2_q[0] in spread_values:
                      if not f2_q in f2_valid_configurations:
                          f2_valid_configurations.append(f2_q)

              valid_configurations[0] = f1_valid_configurations
              valid_configurations[1] = f2_valid_configurations

              if len(valid_configurations[0]) > 0 and len(valid_configurations[1]) > 0 and len(valid_configurations[2]) > 0:
                  pass
              else:
                  return points_for_config, points_f_for_config

              # update the points set
              points_in_sphere2 = []
              for pt in points_in_sphere:
                  q = pt[4]
                  if q[0] == None or q[0] in spread_values:
                      points_in_sphere2.append(pt)

              points_f_in_sphere2 = []
              for pt in points_f_in_sphere:
                  q = pt[4]
                  if q[0] == None or q[0] in spread_values:
                      points_f_in_sphere2.append(pt)

              for pt in points_in_sphere2:
                  if not pt[4] in points_for_config:
                      points_for_config[pt[4]] = [pt]
                  else:
                      points_for_config[pt[4]].append(pt)

#              print "points_in_sphere: %s"%(len(points_in_sphere2))
#              print "configs: %s"%(len(points_for_config))

              for pt in points_f_in_sphere2:
                  if not pt[4] in points_f_for_config:
                      points_f_for_config[pt[4]] = [pt]
                  else:
                      points_f_for_config[pt[4]].append(pt)

#              print "points_f_in_sphere: %s"%(len(points_f_in_sphere2))
#              print "configs_f: %s"%(len(points_f_for_config))

              return points_for_config, points_f_for_config

    def calculateValidOrientations(self, points_for_config, points_f_for_config, vol_obj, pos):
              ori_for_config = {}
              points_for_config2 = {}
              # for each config calculate valid orientations
              for cf in points_for_config:
                  allowed_ori = set()
                  forbidden_ori = set()

                  if cf in points_f_for_config:
                      for f_pt_f in points_f_for_config[cf]:
                          vol_idx = vol_obj.getVolIndex(f_pt_f[1]-pos)
                          vol_sample = vol_obj.vol_samples[vol_idx[0]][vol_idx[1]][vol_idx[2]]
                          forbidden_ori = forbidden_ori.union(set(vol_sample.keys()))

                  for f_pt in points_for_config[cf]:
                      vol_idx = vol_obj.getVolIndex(f_pt[1]-pos)
                      vol_sample = vol_obj.vol_samples[vol_idx[0]][vol_idx[1]][vol_idx[2]]
                      oris_to_check = set(vol_sample.keys())
                      allowed_ori = allowed_ori.union(oris_to_check)

                  ori = allowed_ori.difference(forbidden_ori)
                  if len(ori) > 0:
                      ori_for_config[cf] = ori
                      points_for_config2[cf] = points_for_config[cf]
              return ori_for_config, points_for_config2

    def joinFingersConfigurations(self, points_for_config2, ori_for_config, self_collisions_configs, pos, vol_obj, voxel_grid):
              # return value
              good_grasps = []

              TT_E_H = PyKDL.Frame(pos)

              points_for_config_f1 = {}
              points_for_config_f2 = {}
              points_for_config_f3 = {}

              for cf in points_for_config2:
                  if cf[1] != None:
                      points_for_config_f1[cf] = points_for_config2[cf]
                  elif cf[3] != None:
                      points_for_config_f2[cf] = points_for_config2[cf]
                  elif cf[2] != None:
                      points_for_config_f3[cf] = points_for_config2[cf]
                  else:
                      print "error: wrong hand configuration %s"%(offset)
                      exit(0)

              all_configs = 0
              good_poses = 0
              good_configs = {}
              forbidden_cf_ori = {}
              for cf1 in points_for_config_f1:
                  forbidden_cf_ori[cf1] = set()
              for cf2 in points_for_config_f2:
                  forbidden_cf_ori[cf2] = set()
              for cf3 in points_for_config_f3:
                  forbidden_cf_ori[cf3] = set()

              normals_for_config = {}
              contacts_obj_for_config = {}
              contacts_link_for_config = {}
              normals_link_for_config = {}
              dir_link_for_config = {}
              is_plane_obj_config = {}

              # iterate through all locally possible hand configs
              for cf1 in points_for_config_f1:
                for cf2 in points_for_config_f2:
                  if cf2[0] != cf1[0]:
                      continue
                  for cf3 in points_for_config_f3:
                      all_configs += 1
                      cf = (cf1[0], cf1[1], cf3[2], cf2[3])

                      # eliminate self collision configs
                      if cf in self_collisions_configs:
                          continue

                      # get the intersection of the orientations set for contact of the object with each of the fingers
                      ori_set = ori_for_config[cf1].intersection(ori_for_config[cf2], ori_for_config[cf3])
                      ori_set = ori_set.difference(forbidden_cf_ori[cf1], forbidden_cf_ori[cf2], forbidden_cf_ori[cf3])

                      # perform additional checks for each possible orientation
                      ori_set_ok = []
                      for ori_idx in ori_set:

                          T_E_O = TT_E_H * vol_obj.orientations[ori_idx] * vol_obj.T_H_O
                          TR_E_O = PyKDL.Frame(T_E_O.M)

                          for cfx in [cf1, cf2, cf3]:
                              ori_ok = False
                              if ori_idx in forbidden_cf_ori[cfx]:
                                  break
                              if (cfx,ori_idx) in contacts_link_for_config:
                                  continue
                              for f_pt in points_for_config2[cfx]:
                                  vol_idx = vol_obj.getVolIndex(f_pt[1]-pos)
                                  vol_sample = vol_obj.vol_samples[vol_idx[0]][vol_idx[1]][vol_idx[2]]
                                  if not ori_idx in vol_sample:
                                      continue

                                  norm, type_surf = vol_sample[ori_idx]
                                  normal_obj_E = TR_E_O * norm
                                  # check the contact between two surfaces
                                  if f_pt[3] == 0 and type_surf==0:
                                      if PyKDL.dot(normal_obj_E, f_pt[2]) < -0.8:
                                          ori_ok = True
                                  elif (f_pt[3] == 0 and type_surf==2) or (f_pt[3] == 2 and type_surf==0):
                                      if PyKDL.dot(normal_obj_E, f_pt[2]) < -0.3:
                                          ori_ok = True
                                  elif (f_pt[3] == 0 and type_surf==1) or (f_pt[3] == 1 and type_surf==0):
                                      if PyKDL.dot(normal_obj_E, f_pt[2]) < -0.3:
                                          ori_ok = True
                                  elif f_pt[3] == 1 and type_surf==1:
                                      if PyKDL.dot(normal_obj_E, f_pt[2]) < -0.3:
                                          ori_ok = True

                                  if ori_ok:

                                      if not (cfx,ori_idx) in contacts_link_for_config:
                                          contacts_obj_for_config[(cfx,ori_idx)] = [pos + vol_obj.getVolPoint(vol_idx[0],vol_idx[1],vol_idx[2])]
                                          normals_for_config[(cfx,ori_idx)] = [normal_obj_E]
                                          contacts_link_for_config[(cfx,ori_idx)] = [f_pt[1]]
                                          normals_link_for_config[(cfx,ori_idx)] = [f_pt[2]]
                                          dir_link_for_config[(cfx,ori_idx)] = [f_pt[5]]
                                          is_plane_obj_config[(cfx,ori_idx)] = 0
                                      else:
                                          contacts_obj_for_config[(cfx,ori_idx)].append(pos + vol_obj.getVolPoint(vol_idx[0],vol_idx[1],vol_idx[2]))
                                          contacts_link_for_config[(cfx,ori_idx)].append(f_pt[1])
                                          normals_link_for_config[(cfx,ori_idx)].append(f_pt[2])
                                          dir_link_for_config[(cfx,ori_idx)].append(f_pt[5])
                                          normals_for_config[(cfx,ori_idx)].append(normal_obj_E)

                                      if type_surf == 0:
                                          is_plane_obj_config[(cfx,ori_idx)] += 1


#                                      if not (cfx,ori_idx) in contacts_obj_for_config:
#                                          contacts_obj_for_config[(cfx,ori_idx)] = [pos + vol_obj.getVolPoint(vol_idx[0],vol_idx[1],vol_idx[2])]
#                                      else:
#                                          contacts_obj_for_config[(cfx,ori_idx)].append(pos + vol_obj.getVolPoint(vol_idx[0],vol_idx[1],vol_idx[2]))
#                                      if not (cfx,ori_idx) in normals_for_config:
#                                          normals_for_config[(cfx,ori_idx)] = [normal_obj_E]
#                                      else:
#                                          normals_for_config[(cfx,ori_idx)].append(normal_obj_E)
#                                      is_plane_obj_config[(cfx,ori_idx)] = type_surf==0
                                  else:
                                      forbidden_cf_ori[cfx].add(ori_idx)
                                      break
                              if not ori_ok:
                                  break
                          if not ori_ok:
                              continue

                          # at least one contact should be with planar part of the object
                          if not is_plane_obj_config[(cf1,ori_idx)] and not is_plane_obj_config[(cf2,ori_idx)] and not is_plane_obj_config[(cf3,ori_idx)]:
                              continue

                          # check if there is a collision of the gripper with the other part of the object
                          collision = False
                          for disabled_pt_idx in vol_obj.sampled_points2_obj:
                              pt_O = vol_obj.surface_points_obj[disabled_pt_idx].pos
                              pt_E = TT_E_H * vol_obj.orientations[ori_idx] * vol_obj.T_H_O * pt_O
                              xi, yi, zi = voxel_grid.getPointIndex(pt_E)
                              if xi >= voxel_grid.grid_size[0] or xi < 0 or yi >= voxel_grid.grid_size[1] or yi < 0 or zi >= voxel_grid.grid_size[2] or zi < 0:
                                  continue
                              for pt_gr in voxel_grid.grid[xi][yi][zi]:
                                  if pt_gr[4] == cf1 or pt_gr[4] == cf2 or pt_gr[4] == cf3:
                                      collision = True
                                      break

                          if not collision:
                              ori_set_ok.append(ori_idx)
                              contacts_obj_for_config[(cf,ori_idx)] = (contacts_obj_for_config[(cf1,ori_idx)], contacts_obj_for_config[(cf2,ori_idx)], contacts_obj_for_config[(cf3,ori_idx)])
                              contacts_link_for_config[(cf,ori_idx)] = (contacts_link_for_config[(cf1,ori_idx)], contacts_link_for_config[(cf2,ori_idx)], contacts_link_for_config[(cf3,ori_idx)])
                              normals_for_config[(cf,ori_idx)] = (normals_for_config[(cf1,ori_idx)], normals_for_config[(cf2,ori_idx)], normals_for_config[(cf3,ori_idx)])
                              normals_link_for_config[(cf,ori_idx)] = (normals_link_for_config[(cf1,ori_idx)], normals_link_for_config[(cf2,ori_idx)], normals_link_for_config[(cf3,ori_idx)])
                              dir_link_for_config[(cf,ori_idx)] = (dir_link_for_config[(cf1,ori_idx)], dir_link_for_config[(cf2,ori_idx)], dir_link_for_config[(cf3,ori_idx)])

                      for ori_idx in ori_set_ok:
                          grasp = VolumetricGrasp()

                          grasp.hand_config = cf
                          grasp.obj_ori_idx = ori_idx
                          grasp.contacts_obj = contacts_obj_for_config[(cf,ori_idx)]
                          grasp.contacts_link = contacts_link_for_config[(cf,ori_idx)]
                          grasp.normals_obj = normals_for_config[(cf,ori_idx)]
                          grasp.normals_link = normals_link_for_config[(cf,ori_idx)]
                          grasp.dof_directions = []
                          for cf_idx in range(len(dir_link_for_config[(cf,ori_idx)])):
                              total_d_plus = 0
                              total_d_minus = 0
                              total_d_zero = 0
                              for d in dir_link_for_config[(cf,ori_idx)][cf_idx]:
                                  if d == 1:
                                      total_d_plus += 1
                                  elif d == -1:
                                      total_d_minus += 1
                                  elif d == 0:
                                      total_d_zero += 1
                                  else:
                                      print "ERROR: joinFingersConfigurations: direction: %s"%(d)
                              total_d_idx = np.argmax( (total_d_plus, total_d_minus, total_d_zero) )
                              if total_d_idx == 0:
                                  total_d = 1
                              elif total_d_idx == 1:
                                  total_d = -1
                              else:
                                  total_d = 0
                              grasp.dof_directions.append(total_d)

                          if grasp.dof_directions[0] == 0 and grasp.dof_directions[1] == 0 and grasp.dof_directions[2] == 0:
                              continue
                          grasp.obj_pos_E = pos
                          grasp.calculateWrenches()
                          grasp.calculateWrenchesOnLink()
                          good_grasps.append(grasp)

              return good_grasps

    def getGraspsForPosition(self, pos, vol_obj, voxel_grid, self_collisions_configs, gripper_model, normals_sphere_contacts):

                points_in_sphere, points_f_in_sphere, valid_configurations = voxel_grid.getPointsAtPoint(pos, vol_obj.vol_radius)

#                print "points_in_sphere: %s  %s  %s"%(len(points_in_sphere), len(points_f_in_sphere), len(valid_configurations))
                points_for_config, points_f_for_config = self.sortPointsByConfig(points_in_sphere, points_f_in_sphere, valid_configurations)

                if len(points_for_config) == 0:
                    return None


                print "calculating valid orientations..."
                ori_for_config, points_for_config2 = self.calculateValidOrientations(points_for_config, points_f_for_config, vol_obj, pos)
                print "done"

                points_for_config = None
                points_f_for_config = None

                # visualization
#                if False:
#                    TT_E_H = PyKDL.Frame(pos)
#                    self.visualize1(T_W_E * TT_E_H, T_H_O, joint_map, points_for_config2, ori_for_config, orientations, sp_configs, f1_configs, f2_configs, f3_configs, self.pub_marker)
#                    exit(0)

                print "joining fingers configurations..."
                good_grasps = self.joinFingersConfigurations(points_for_config2, ori_for_config, self_collisions_configs, pos, vol_obj, voxel_grid)
                print "done."

                # initial selection of grasps based on contact forces analysis
                print "initial selection of grasps based on contact forces analysis..."
                good_grasps2 = []
                for grasp in good_grasps:
                    cf = grasp.hand_config
                    ori = grasp.obj_ori_idx

                    grasp_ok = True
                    for n in normals_sphere_contacts:
                        min_f = None
                        max_f = None
                        min_t = None
                        max_t = None
                        for cf_idx in range(len(grasp.wrenches)):
                            for wr in grasp.wrenches[cf_idx]:
                                f = PyKDL.dot(PyKDL.Vector(wr[0], wr[1], wr[2]), n)
                                t = PyKDL.dot(PyKDL.Vector(wr[3], wr[4], wr[5]), n)
                                if min_f == None or min_f > f:
                                    min_f = f
                                if max_f == None or max_f < f:
                                    max_f = f
                                if min_t == None or min_t > t:
                                    min_t = t
                                if max_t == None or max_t < t:
                                    max_t = t
                        if max_f < 0.0 or min_f > 0.0 or max_t < 0.0 or min_t > 0.0:
                            grasp_ok = False
                            break
                    if grasp_ok:
                        good_grasps2.append(grasp)

                good_grasps = good_grasps2

                # initial selection of grasps based on contact forces analysis
                print "initial selection of grasps based on contact forces analysis (second pass)..."
                good_grasps2 = []
                for grasp in good_grasps:
                        grasp_ok = True
                        for n in normals_sphere_contacts:
                            min_f = None
                            max_f = None
                            min_t = None
                            max_t = None
                            for cf_idx in range(len(grasp.wrenches_link)):
                                for wr in grasp.wrenches_link[cf_idx]:
                                    f = PyKDL.dot(PyKDL.Vector(wr[0], wr[1], wr[2]), n)
                                    t = PyKDL.dot(PyKDL.Vector(wr[3], wr[4], wr[5]), n)
                                    if min_f == None or min_f > f:
                                        min_f = f
                                    if max_f == None or max_f < f:
                                        max_f = f
                                    if min_t == None or min_t > t:
                                        min_t = t
                                    if max_t == None or max_t < t:
                                        max_t = t
                            if max_f < 0.0 or min_f > 0.0 or max_t < 0.0 or min_t > 0.0:
                                grasp_ok = False
                                break
                        if grasp_ok:
                            good_grasps2.append(grasp)

                return good_grasps2

    def graspingThread(self, args, queue):
        pos, vol_obj, voxel_grid, self_collisions_configs, gripper_model, normals_sphere_contacts = args
        good_grasps = self.getGraspsForPosition(pos, vol_obj, voxel_grid, self_collisions_configs, gripper_model, normals_sphere_contacts)
        if good_grasps == None:
            good_grasps = []
        queue.put(good_grasps)

    def spin(self):

#        y_values = np.linspace(-0.06, 0.06, 28)
#        print y_values
#        exit(0)


        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

        #
        # Init Openrave
        #
        parser = OptionParser(description='Openrave Velma interface')
        OpenRAVEGlobalArguments.addOptions(parser)
        (options, leftargs) = parser.parse_args()
        self.env = OpenRAVEGlobalArguments.parseAndCreate(options)#,defaultviewer=True)

        self.openrave_robot = self.env.ReadRobotXMLFile('robots/barretthand_ros.robot.xml')

        joint_names = []
        print "active joints:"
        for j in self.openrave_robot.GetJoints():
            joint_names.append(j.GetName())
            print j

        print "passive joints:"
        for j in self.openrave_robot.GetPassiveJoints():
            joint_names.append(j.GetName())
            print j

        # ODE does not support distance measure
        self.env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)

        self.env.Add(self.openrave_robot)

        vertices, faces = surfaceutils.readStl("klucz_gerda_ascii.stl", scale=1.0)
        self.addTrimesh("object", vertices, faces)

        #
        # definition of the expected external wrenches
        #
        ext_wrenches = []

        # origin of the external wrench (the end point of the key)
        wr_orig = PyKDL.Vector(0.039, 0.0, 0.0)

        for i in range(8):
            # expected force at the end point
            force = PyKDL.Frame(PyKDL.Rotation.RotX(float(i)/8.0 * 2.0 * math.pi)) * PyKDL.Vector(0,1,0)
            # expected torque at the com
            torque = wr_orig * force
            ext_wrenches.append(PyKDL.Wrench(force, torque))

            # expected force at the end point
            force = PyKDL.Frame(PyKDL.Rotation.RotX(float(i)/8.0 * 2.0 * math.pi)) * PyKDL.Vector(-1,1,0)
            # expected torque at the com
            torque = wr_orig * force
            ext_wrenches.append(PyKDL.Wrench(force, torque))

#        self.updatePose("object", self.T_W_O)

        with open('barret_hand_openrave2ros_joint_map2.txt', 'r') as f:
            lines = f.readlines()
            joint_map = {}
            for line in lines:
                line_s = line.split()
                if len(line_s) == 2:
                    joint_map[line_s[0]] = line_s[1]
                elif len(line_s) != 1:
                    print "error in joint map file"
                    exit(0)

        print joint_map

        self.pub_js = rospy.Publisher("/joint_states", JointState)

        #
        # create normals set for the contact point forces verification procedure
        #
        normals_sphere_contacts = velmautils.generateNormalsSphere(20.0/180.0*math.pi)
        print "normals_sphere_contacts: %s"%(len(normals_sphere_contacts))

        #
        # create volumetric model of the key handle
        #
        print "sampling the surface of the object..."
        vertices_obj, faces_obj = surfaceutils.readStl("klucz_gerda_ascii.stl", scale=1.0)

        T_O_H = PyKDL.Frame(PyKDL.Vector(-0.0215,0,0))
        T_H_O = T_O_H.Inverse()

        vol_obj = volumetricutils.VolumetricModel(vol_radius = 0.022, vol_samples_count = 16, T_H_O = T_H_O, orientations_angle = 10.0/180.0*math.pi, vertices_obj = vertices_obj, faces_obj = faces_obj)

        vol_map_filename = "vol_map.txt"
        if False:
            print "generating volumetric map (%s iterations)..."%(len(vol_obj.orientations) * len(vol_obj.surface_points_obj))
            vol_obj.generate()
            print "done."
            vol_obj.save(vol_map_filename)
        else:
            print "reading the volumetric map from file %s"%(vol_map_filename)
            vol_obj.load(vol_map_filename)

        # test volumetric model
        if False:
            vol_obj.test1(self.pub_marker, PyKDL.Frame(PyKDL.Vector(0,0,0.5)))
            exit(0)

        #
        # sample the surface of the gripper link
        #
        self.grasper = interfaces.Grasper(self.openrave_robot,friction=1.0 )
        gripper_model = GripperModel(self.openrave_robot, self.grasper)

        print "done."

        checked_links = [
        "right_HandFingerOneKnuckleThreeLink",
        "right_HandFingerThreeKnuckleThreeLink",
        "right_HandFingerTwoKnuckleThreeLink",
        ]

        T_W_E = OpenraveToKDL(self.openrave_robot.GetLink("right_HandPalmLink").GetTransform())
        TR_W_E = PyKDL.Frame(T_W_E.M)
        T_E_W = T_W_E.Inverse()

        if False:
            if False:
                for fi in range(len(f1_configs)):
                    cf = [5, fi, 0, 0]
                    self.openrave_robot.SetDOFValues([sp_configs[cf[0]]/180.0*math.pi,f1_configs[cf[1]]/180.0*math.pi,f3_configs[cf[2]]/180.0*math.pi,f2_configs[cf[3]]/180.0*math.pi])
                    for i in range(0, 2):
                        # update the gripper visualization in ros
                        js = JointState()
                        js.header.stamp = rospy.Time.now()
                        for jn in joint_map:
                            js.name.append(joint_map[jn])
                            js.position.append(self.openrave_robot.GetJoint(jn).GetValue(0))
                        self.pub_js.publish(js)
                        rospy.sleep(0.1)
                    raw_input("x")
                exit(0)

            # create the voxel map of the points from the inside of the gripper links
            # create the voxel map of the points from the surface of the gripper
            points, points_forbidden = gripper_model.generateSamples()

            print "points: %s"%(len(points))

#            m_id = 0
#            for i in range(3000):
#                m_id = pub_marker.publishSinglePointMarker(T_W_E*points[i][1], m_id, r=0, g=1, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=None)
#                rospy.sleep(0.001)

#            exit(0)
            #
            # check self collisions of the gripper
            #
            self_collision_set_filename = "self_collision_40.txt"
            self_collisions_configs = set()
            if False:
                print "checking self collision for all configurations..."
                self_collisions_configs = gripper_model.generateSelfCollisionData()
                print "done."
                self.saveSelfCollisionData(self_collision_set_filename, self_collisions_configs)
            else:
                self_collisions_configs = self.loadSelfCollisionData(self_collision_set_filename)


            voxel_grid = volumetricutils.VoxelGrid(0.005)
            voxel_grid.build(points, points_forbidden)
            print "dim_min: %s"%(voxel_grid.dim_min)
            print "dim_max: %s"%(voxel_grid.dim_max)
            print "voxel_map_size: %s %s %s"%(voxel_grid.grid_size[0], voxel_grid.grid_size[1], voxel_grid.grid_size[2])
            print "voxels: %s"%(voxel_grid.grid_size[0]*voxel_grid.grid_size[1]*voxel_grid.grid_size[2])
            print "max_points_in_voxel: %s"%(voxel_grid.max_points_in_voxel)
            print "max_points_in_voxel (f): %s"%(voxel_grid.max_points_in_voxel_f)


            init_pos = PyKDL.Vector(0,0,0.14)
            print "pos: %s"%(init_pos)

            # the BarrettHand gripper is symmetrical in x axis
            offsets = []
#            for x in np.linspace(0.0, 0.02, 5):
#              for y in np.linspace(-0.06, 0.06, 28):
#                for z in np.linspace(-0.02, 0.06, 19):
            for x in np.linspace(0.0, 0.02, 10):
              for y in np.linspace(-0.06, 0.06, 60):
                for z in np.linspace(-0.02, 0.06, 40):
                  offsets.append(PyKDL.Vector(x,y,z))

            print "number of candidate positions: %s"%(len(offsets))
            valid_positions = []

            # multiprocessing
            good_grasps = 0
            threads_queues = [None, None, None]#, None]
            offset_idx = 644
            stop = False
            while True:
                stopped_threads = 0
                for th_idx in range(len(threads_queues)):
                    start_thread = False
                    if threads_queues[th_idx] == None:
                        if stop:
                            stopped_threads += 1
                        else:
                            start_thread = True
                            threads_queues[th_idx] = Queue()
                    elif not threads_queues[th_idx].empty():
                        if stop:
                            grasps = threads_queues[th_idx].get()
                            good_grasps += len(grasps)
                            print "thread %s grasps: %s"%(th_idx, len(grasps))
                            print "total grasps: %s"%(good_grasps)
                            with open("grasps05.txt", 'a') as f:
                                for grasp in grasps:
                                    grasp.saveToFile(f)

                            threads_queues[th_idx] = None
                        else:
                            start_thread = True
                            grasps = threads_queues[th_idx].get()
                            good_grasps += len(grasps)
                            print "thread %s grasps: %s"%(th_idx, len(grasps))
                            print "total grasps: %s"%(good_grasps)
                            with open("grasps05.txt", 'a') as f:
                                for grasp in grasps:
                                    grasp.saveToFile(f)
                    if start_thread:
                        if stop:
                            print "ERROR: start_thread and stop"
                        print "starting thread %s for %s"%(th_idx, offset_idx)
                        pos = init_pos + offsets[offset_idx]
                        args = (pos, vol_obj, voxel_grid, self_collisions_configs, gripper_model, normals_sphere_contacts)
                        p = Process(target=self.graspingThread, args=(args,threads_queues[th_idx]))
                        p.start()

                        offset_idx += 1
                        if offset_idx >= len(offsets):
                            stop = True
                if stopped_threads == len(threads_queues):
                    break
                rospy.sleep(0.1)

            print "Ended."
#            with open("grasps02.txt", 'w') as f:
#                for grasp in good_grasps:
#                    grasp.saveToFile(f)
            exit(0)

        else:
            good_grasps = []
            index = 0
            print "reading grasps from file..."
            with open("grasps04.txt", 'r') as f:
                while True:
                    grasp = VolumetricGrasp()
                    if grasp.loadFromFileFast(f):
                        good_grasps.append(grasp)
                    else:
                        break
                    index += 1
                    if index % 100000 == 0:
                        print index

        print "good grasps: %s"%(len(good_grasps))

        def getDirectionIndex(n):
            min_angle = -45.0/180.0*math.pi
            angle_range = 90.0/180.0*math.pi
            angles_count = 2
            angles_count2 = angles_count * angles_count
            max_indices = angles_count2 * 6
            if abs(n.x()) > abs(n.y()) and abs(n.x()) > abs(n.z()):
                    if n.x() > 0:
                        sec = 0
                        a1 = math.atan2(n.y(), n.x())
                        a2 = math.atan2(n.z(), n.x())
                    else:
                        sec = 1
                        a1 = math.atan2(n.y(), -n.x())
                        a2 = math.atan2(n.z(), -n.x())
            elif abs(n.y()) > abs(n.x()) and abs(n.y()) > abs(n.z()):
                if n.y() > 0:
                    sec = 2
                    a1 = math.atan2(n.x(), n.y())
                    a2 = math.atan2(n.z(), n.y())
                else:
                    sec = 3
                    a1 = math.atan2(n.x(), -n.y())
                    a2 = math.atan2(n.z(), -n.y())
            else:
                if n.z() > 0:
                    sec = 4
                    a1 = math.atan2(n.x(), n.z())
                    a2 = math.atan2(n.y(), n.z())
                else:
                    sec = 5
                    a1 = math.atan2(n.x(), -n.z())
                    a2 = math.atan2(n.y(), -n.z())

            a1i = int(angles_count*(a1-min_angle)/angle_range)
            a2i = int(angles_count*(a2-min_angle)/angle_range)
            if a1i < 0:
#                print sec, a1i, a2i
                a1i = 0
            if a1i >= angles_count:
                a1i = angles_count-1
            if a2i < 0:
#                print sec, a1i, a2i
                a2i = 0
            if a2i >= angles_count:
#                print sec, a1i, a2i
                a2i = angles_count-1
            return sec * angles_count2 + a1i * angles_count + a2i

        grasps_grid = volumetricutils.GraspsVoxelGrid(0.002)
        grasps_grid.build(good_grasps)
        grasps_neighborhood = []
        indices_ok = []
        
        for grasp_idx in range(len(good_grasps)):
            grasp = good_grasps[grasp_idx]
            grasp_cf = gripper_model.getAnglesForConfigIdx(grasp.hand_config)
            close_grasps = grasps_grid.getPointsAtPoint(grasp.obj_pos_E, 0.003)
#            neighborhood_size = 0
            pos_indices = set()
            rot_indices = set()
            errors = []
            for close_gr in close_grasps:
                # check if the hand configuration is close
                if max( np.fabs( grasp_cf - gripper_model.getAnglesForConfigIdx(close_gr.hand_config) ) ) > 20.0/180.0*math.pi:
                    continue

                diff = PyKDL.diff(vol_obj.orientations[close_gr.obj_ori_idx], vol_obj.orientations[grasp.obj_ori_idx])
                pos_diff = close_gr.obj_pos_E - grasp.obj_pos_E
                rot_diff = diff.rot
                if rot_diff.Norm() > 20.0/180.0*math.pi or math.isnan(rot_diff.Norm()):
                    continue
                
                if grasp.dof_directions[0] * close_gr.dof_directions[0] < 0:
                    continue
                if grasp.dof_directions[1] * close_gr.dof_directions[1] < 0:
                    continue
                if grasp.dof_directions[2] * close_gr.dof_directions[2] < 0:
                    continue
                if pos_diff.Norm() > 0.0001:
                    pos_indices.add( getDirectionIndex(pos_diff) )
                    errors.append((pos_diff, rot_diff))
                if rot_diff.Norm() > 0.0001:
                    rot_indices.add( getDirectionIndex(rot_diff) )
            print (grasp_idx, len(errors), len(pos_indices), len(rot_indices))
            if len(errors) > 1 and len(rot_indices) > 4:
                indices_ok.append(grasp_idx)


#        self.hand_config = None
#        self.obj_ori_idx = None
#        self.contacts_obj = None
#        self.contacts_link = None
#        self.normals_obj = None
#        self.contacts_obj_reduced = None
#        self.wrenches = None
#        self.obj_pos_E = None
#        self.normals_link = None
#        self.dof_directions = None
#        self.contacts_link_reduced = None
#        self.wrenches_link = None


        exit(0)


        ##############################################################################
        # Generate sample data
#        centers = [[1, 1], [-1, -1], [1, -1]]
#        X, labels_true = make_blobs(n_samples=5, centers=centers, cluster_std=0.4,
#                            random_state=0)

        grasps_dir = {}
        for grasp in good_grasps:
            if not grasp.dof_directions in grasps_dir:
                grasps_dir[grasp.dof_directions] = [grasp]
            else:
                grasps_dir[grasp.dof_directions].append(grasp)

        print "closing directions: %s"%(len(grasps_dir))


        for dof_dir in grasps_dir:
            X = []
            grasps = grasps_dir[dof_dir]
            for grasp in grasps:
                rot = vol_obj.orientations[grasp.obj_ori_idx].M.GetRot()
#                X.append( [grasp.hand_config[0], grasp.hand_config[1], grasp.hand_config[2], grasp.hand_config[3], grasp.obj_pos_E[0], grasp.obj_pos_E[1], grasp.obj_pos_E[2], rot[0], rot[1], rot[2]] )
                X.append( [grasp.obj_pos_E[0], grasp.obj_pos_E[1], grasp.obj_pos_E[2], rot[0], rot[1], rot[2]] )



            def graspDistanceMeasure(x, y):
                pos_dist = (PyKDL.Vector(x[0], x[1], x[2]) - PyKDL.Vector(y[0], y[1], y[2])).Norm()
                rot_dist = (PyKDL.Vector(x[3], x[4], x[5]) - PyKDL.Vector(y[3], y[4], y[5])).Norm()
                # 0.005m ~ 20deg
                dist = pos_dist / 0.005 + rot_dist / (20.0/180.0*math.pi)
                return dist

            # get two nonzero values and their indices for each feature
            nonzero_features = []
            features = len(X[0])
            for i in range(features):
                nonzero_features.append([])
            for sample_idx in range(len(X)):
                sample = X[sample_idx]
                for feature_idx in range(features):
                    if len(nonzero_features[feature_idx]) < 2 and abs(sample[feature_idx]) > 0.001:
                        nonzero_features[feature_idx].append((sample[feature_idx], feature_idx))

#            X_first = (X[0][0], X[0][1], X[0][2], X[0][3], X[0][4], X[0][5])
            X = StandardScaler().fit_transform(X)

#            a1 = nonzero_features[feature_idx][0][0]
#            a1_ = X[nonzero_features[feature_idx][0][1]]
#            a2 = nonzero_features[feature_idx][1][0]
#            a2_ = X[nonzero_features[feature_idx][1][1]]
#            c = a2_ - 
#            transforms = ()

#            db = DBSCAN(eps=0.5, min_samples=10, metric=graspDistanceMeasure).fit(X)
#            core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
#            core_samples_mask[db.core_sample_indices_] = True
#            labels = db.labels_

            labels = db = DBSCAN(eps=0.5, min_samples=10, metric=graspDistanceMeasure).fit_predict(X)
#, metric=graspDistanceMeasure
            # Number of clusters in labels, ignoring noise if present.
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
            print "dir: %s %s %s   X: %s    n_clusters: %s"%(dof_dir[0], dof_dir[1], dof_dir[2], len(X), n_clusters_)

            grasps_labeled = {}
            for gr_idx in range(len(labels)):
                label = labels[gr_idx]
                if not label in grasps_labeled:
                    grasps_labeled[label] = [grasps[gr_idx]]
                else:
                    grasps_labeled[label].append(grasps[gr_idx])

            for label in grasps_labeled:
                print "label: %s    grasps: %s"%(label, len(grasps_labeled[label]))
                if label < 0:
                    continue
                for grasp in grasps_labeled[label]:
                    self.visualizeGrasp(T_W_E, grasp, vol_obj, gripper_model, self.pub_marker, joint_map)
                    ch = raw_input("press [n] and [ENTER] for next label...")
                    if ch == 'n' or ch == 'N':
                        break

        # sort grasps by their positions
        grasps_pos = {}
        max_grasps_at_pos = 0
        for grasp in good_grasps:
            pos_idx = (int(grasp.obj_pos_E[0]*1000.0), int(grasp.obj_pos_E[1]*1000.0), int(grasp.obj_pos_E[2]*1000.0))
            if not pos_idx in grasps_pos:
                grasps_pos[pos_idx] = [grasp]
            else:
                grasps_pos[pos_idx].append(grasp)
            if len(grasps_pos[pos_idx]) > max_grasps_at_pos:
                max_grasps_at_pos = len(grasps_pos[pos_idx])

        print "max_grasps_at_pos: %d"%(max_grasps_at_pos)
        m_id = 0
        for pos_idx in grasps_pos:
            pos = PyKDL.Vector(float(pos_idx[0])/1000.0, float(pos_idx[1])/1000.0, float(pos_idx[2])/1000.0)
            size = 0.002 * float(len(grasps_pos[pos_idx]))/max_grasps_at_pos
            m_id = pub_marker.publishSinglePointMarker(pos, m_id, r=0, g=1, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(size, size, size), T=T_W_E)
            rospy.sleep(0.001)
        exit(0)

        self.visualize2(T_W_E, good_grasps, vol_obj, gripper_model, self.pub_marker, joint_map)

if __name__ == '__main__':

    rospy.init_node('grasp_leanring')

    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)

    task.spin()

