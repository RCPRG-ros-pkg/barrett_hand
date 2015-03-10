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

    def updatePose(self, name, T_Br_Bo):
        with self.env:
            body = self.env.GetKinBody(name)
            if body != None:
                body.SetTransform(self.KDLToOpenrave(T_Br_Bo))
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

    def getGraspQHull(self, points):
        try:
            planes, faces, triangles = self.grasper.ConvexHull(np.array(points), returnplanes=True,returnfaces=False,returntriangles=False)
        except AttributeError as e1:
            print e1
            return None
        return planes

    def contactToWrenches(self, pos, normal, friction, Nconepoints):
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

    def generateGWS(self, contacts, friction):
        qhullpoints = []
        qhullpoints_contact_idx = []
        contact_idx = 0
        for c in contacts:
            p = PyKDL.Vector(c[0], c[1], c[2])
            nz = PyKDL.Vector(c[3], c[4], c[5])
            wrs = self.contactToWrenches(p, nz, friction, 6)
            for wr in wrs:
                qhullpoints.append([wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]])
                qhullpoints_contact_idx.append(contact_idx)
            contact_idx += 1

        qhullplanes = self.getGraspQHull(qhullpoints)
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

    def generateSelfCollisionData(self, sp_configs, f1_configs, f2_configs, f3_configs):
        self_collisions_configs = set()
        for sp_idx in range(len(sp_configs)):
            print "%s / %s"%(sp_idx, len(sp_configs))
            sp = sp_configs[sp_idx]
            for f1_idx in range(len(f1_configs)):
                f1 = f1_configs[f1_idx]
                for f2_idx in range(len(f2_configs)):
                    f2 = f2_configs[f2_idx]
                    for f3_idx in range(len(f3_configs)):
                        f3 = f3_configs[f3_idx]
                        self.openrave_robot.SetDOFValues([sp/180.0*math.pi, f1/180.0*math.pi, f3/180.0*math.pi, f2/180.0*math.pi])
                        if self.openrave_robot.CheckSelfCollision():
                            self_collisions_configs.add( (sp_idx, f1_idx, f3_idx, f2_idx) )
        return self_collisions_configs

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

    def spin(self):
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


        if True:
            #
            # generate the set of orientations
            #
            normals_sphere = velmautils.generateNormalsSphere(10.0/180.0*math.pi)

            def getNormalIndex(n):
                max_dot = None
                max_idx = None
                normal_idx = 0
                for normal in normals_sphere:
                    dot = PyKDL.dot(normal, n)
                    if max_dot == None or max_dot < dot:
                        max_dot = dot
                        max_idx = normal_idx
                    normal_idx += 1
                return max_idx

            normals_sphere_pos = []
            for n in normals_sphere:
                if n.x() > 0 and n.y() > 0 and n.z() > 0:
                    normals_sphere_pos.append(n)
            print "normals_sphere: %s"%(len(normals_sphere))
            print "normals_sphere_pos: %s"%(len(normals_sphere_pos))
            orientations = velmautils.generateFramesForNormals(10.0/180.0*math.pi, normals_sphere)
            orientations2 = []
            for ori in orientations:
                x_axis = ori * PyKDL.Vector(1,0,0)
                if x_axis.z() > 0.0:
                    orientations2.append(ori)
            orientations = orientations2
            orientations = {}
            for ori_idx in range(len(orientations2)):
                orientations[ori_idx] = orientations2[ori_idx]
            print "orientations set size: %s"%(len(orientations))
            T_O_H = PyKDL.Frame(PyKDL.Vector(-0.0215,0,0))
            T_H_O = T_O_H.Inverse()

            # visualization of orientations
            if False:
                m_id = 0
                for fr in orientations:
                    T_W_O = PyKDL.Frame(PyKDL.Vector(0,0,0.5)) * fr * T_H_O
                    # publish the mesh of the object
                    m_id = self.pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)
                raw_input("Press ENTER to continue...")
                exit(0)

            #
            # create normals set for the contact point forces verification procedure
            #
            normals_sphere_contacts = velmautils.generateNormalsSphere(20.0/180.0*math.pi)
            print "normals_sphere_contacts: %s"%(len(normals_sphere_contacts))

            #
            # object
            #
            print "sampling the surface of the object..."
            vertices_obj, faces_obj = surfaceutils.readStl("klucz_gerda_ascii.stl", scale=1.0)
            surface_points_obj = surfaceutils.sampleMeshDetailedRays(vertices_obj, faces_obj, 0.0015)
            print "surface of the object has %s points"%(len(surface_points_obj))

            # disallow contact with the surface points beyond the key handle
            for p in surface_points_obj:
                if p.pos.x() > -0.005:
                    p.allowed = False

            surface_points_obj_init = []
            surface_points2_obj_init = []
            for sp in surface_points_obj:
                if sp.allowed:
                    surface_points_obj_init.append(sp)
                else:
                    surface_points2_obj_init.append(sp)

            print "generating a subset of surface points of the object..."

            while True:
                p_idx = random.randint(0, len(surface_points_obj)-1)
                if surface_points_obj[p_idx].allowed:
                    break
            p_dist = 0.003

            sampled_points_obj = []
            while True:
                sampled_points_obj.append(p_idx)
                surface_points_obj2 = []
                for sp in surface_points_obj_init:
                    if (sp.pos-surface_points_obj[p_idx].pos).Norm() > p_dist:
                        surface_points_obj2.append(sp)
                if len(surface_points_obj2) == 0:
                    break
                surface_points_obj_init = surface_points_obj2
                p_idx = surface_points_obj_init[0].id

            print "subset size: %s"%(len(sampled_points_obj))

            print "generating a subset of other surface points of the object..."

            p_dist2 = 0.006

            while True:
                p_idx = random.randint(0, len(surface_points_obj)-1)
                if not surface_points_obj[p_idx].allowed:
                    break

            sampled_points2_obj = []
            while True:
                sampled_points2_obj.append(p_idx)
                surface_points2_obj2 = []
                for sp in surface_points2_obj_init:
                    if (sp.pos-surface_points_obj[p_idx].pos).Norm() > p_dist2:
                        surface_points2_obj2.append(sp)
                if len(surface_points2_obj2) == 0:
                    break
                surface_points2_obj_init = surface_points2_obj2
                p_idx = surface_points2_obj_init[0].id

            # test volumetric model
            if False:
                for pt_idx in sampled_points2_obj:
                    pt = surface_points_obj[pt_idx]
                    m_id = self.pub_marker.publishSinglePointMarker(pt.pos, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                    rospy.sleep(0.001)

                for pt_idx in sampled_points_obj:
                    pt = surface_points_obj[pt_idx]
                    m_id = self.pub_marker.publishSinglePointMarker(pt.pos, m_id, r=0, g=1, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                    rospy.sleep(0.001)

            print "subset size: %s"%(len(sampled_points2_obj))

            print "calculating surface curvature at sampled points of the obj..."
            m_id = 0
            planes = 0
            edges = 0
            points = 0
            for pt_idx in range(len(surface_points_obj)):#sampled_points_obj
                indices, nx, pc1, pc2 = surfaceutils.pclPrincipalCurvaturesEstimation(surface_points_obj, pt_idx, 5, 0.003)
#                m_id = self.pub_marker.publishVectorMarker(self.T_W_O * surface_points_obj[pt_idx].pos, self.T_W_O * (surface_points_obj[pt_idx].pos + nx*0.004), m_id, 1, 0, 0, frame='world', namespace='default', scale=0.0002)
#                m_id = self.pub_marker.publishVectorMarker(self.T_W_O * surface_points_obj[pt_idx].pos, self.T_W_O * (surface_points_obj[pt_idx].pos + surface_points_obj[pt_idx].normal*0.004), m_id, 0, 0, 1, frame='world', namespace='default', scale=0.0002)
                surface_points_obj[pt_idx].frame = PyKDL.Frame(PyKDL.Rotation(nx, surface_points_obj[pt_idx].normal * nx, surface_points_obj[pt_idx].normal), surface_points_obj[pt_idx].pos)
                surface_points_obj[pt_idx].pc1 = pc1
                surface_points_obj[pt_idx].pc2 = pc2
                if pc1 < 0.2:
                    surface_points_obj[pt_idx].is_plane = True
                    surface_points_obj[pt_idx].is_edge = False
                    surface_points_obj[pt_idx].is_point = False
                    planes += 1
                elif pc2 < 0.2:
                    surface_points_obj[pt_idx].is_plane = False
                    surface_points_obj[pt_idx].is_edge = True
                    surface_points_obj[pt_idx].is_point = False
                    edges += 1
                else:
                    surface_points_obj[pt_idx].is_plane = False
                    surface_points_obj[pt_idx].is_edge = False
                    surface_points_obj[pt_idx].is_point = True
                    points += 1

            print "obj planes: %s  edges: %s  points: %s"%(planes, edges, points)

            #
            # create volumetric model of the key handle
            #
            vol_obj = volumetricutils.VolumetricModel(vol_radius = 0.022, vol_samples_count = 16, T_H_O = T_H_O)
            vol_map_filename = "vol_map.txt"
            if False:
                print "generating volumetric map (%s iterations)..."%(len(orientations) * len(surface_points_obj))
                vol_obj.generate(orientations, surface_points_obj)
                print "done."
                vol_obj.save(vol_map_filename)
            else:
                print "reading the volumetric map from file %s"%(vol_map_filename)
                vol_obj.load(vol_map_filename)

            # test volumetric model
            if False:
                vol_obj.test1(self.pub_marker, orientations, PyKDL.Frame(PyKDL.Vector(0,0,0.5)))
                exit(0)

            #
            # link
            #
            link = self.openrave_robot.GetLink("right_HandFingerOneKnuckleThreeLink")
            col = link.GetCollisionData()
            vertices = col.vertices
            faces = col.indices

            print "sampling the surface..."
            surface_points = surfaceutils.sampleMeshDetailedRays(vertices, faces, 0.0015)
            print "surface has %s points"%(len(surface_points))

            surface_points_init = []
            for sp in surface_points:
                surface_points_init.append(sp)

            p_idx = random.randint(0, len(surface_points)-1)
            p_dist = 0.004

            print "generating a subset of surface points..."

            sampled_points = []
            while True:
                sampled_points.append(p_idx)
                surface_points2 = []
                for sp in surface_points_init:
                    if (sp.pos-surface_points[p_idx].pos).Norm() > p_dist:
                        surface_points2.append(sp)
                if len(surface_points2) == 0:
                    break
                surface_points_init = surface_points2
                p_idx = surface_points_init[0].id

            print "subset size: %s"%(len(sampled_points))


            for pt_idx in sampled_points:
                indices, nx, pc1, pc2 = surfaceutils.pclPrincipalCurvaturesEstimation(surface_points, pt_idx, 5, 0.003)
#                m_id = self.pub_marker.publishVectorMarker(self.T_W_O * surface_points[pt_idx].pos, self.T_W_O * (surface_points[pt_idx].pos + nx*0.004), m_id, 1, 0, 0, frame='world', namespace='default', scale=0.0002)
#                m_id = self.pub_marker.publishVectorMarker(self.T_W_O * surface_points[pt_idx].pos, self.T_W_O * (surface_points[pt_idx].pos + surface_points[pt_idx].normal*0.004), m_id, 0, 0, 1, frame='world', namespace='default', scale=0.0002)

                surface_points[pt_idx].frame = PyKDL.Frame(PyKDL.Rotation(nx, surface_points[pt_idx].normal * nx, surface_points[pt_idx].normal), surface_points[pt_idx].pos)
                surface_points[pt_idx].pc1 = pc1
                surface_points[pt_idx].pc2 = pc2
                if pc1 < 0.2:
                    surface_points[pt_idx].is_plane = True
                    surface_points[pt_idx].is_edge = False
                    surface_points[pt_idx].is_point = False
                elif pc2 < 0.2:
                    surface_points[pt_idx].is_plane = False
                    surface_points[pt_idx].is_edge = True
                    surface_points[pt_idx].is_point = False
                else:
                    surface_points[pt_idx].is_plane = False
                    surface_points[pt_idx].is_edge = False
                    surface_points[pt_idx].is_point = True



            interior_filename = "link_interior_points.txt"

            print "generating a set of interior points..."
            margin = 0.002
#            vertices_min, vertices_max = velmautils.getMeshBB(vertices, faces)

            points_in_link = []
            for pt_idx in sampled_points:
                if surface_points[pt_idx].pc1 > 0.2:
                    continue
                points_in_link.append(surface_points[pt_idx].pos - surface_points[pt_idx].normal*margin)

            print "points_in_link: %s"%(len(points_in_link))

            print "done."

            checked_links = [
            "right_HandFingerOneKnuckleThreeLink",
            "right_HandFingerThreeKnuckleThreeLink",
            "right_HandFingerTwoKnuckleThreeLink",
            ]

            T_W_E = self.OpenraveToKDL(self.openrave_robot.GetLink("right_HandPalmLink").GetTransform())
            TR_W_E = PyKDL.Frame(T_W_E.M)
            T_E_W = T_W_E.Inverse()

            #
            # discretize the configuration of the gripper
            #
            sp_configs = []
            f1_configs = []
            f2_configs = []
            f3_configs = []
            for sp in np.linspace(0.01, 179.99, 12):
                sp_configs.append(sp)
            for fx in np.linspace(80.0, 130.0, 20):
                f1_configs.append(fx)
                f2_configs.append(fx)
                f3_configs.append(fx)

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
            points = []
            points_forbidden = []
            for sp_idx in range(len(sp_configs)):
              sp = sp_configs[sp_idx]
              for f1_idx in range(len(f1_configs)):
                f1 = f1_configs[f1_idx]
                self.openrave_robot.SetDOFValues([sp/180.0*math.pi, f1/180.0*math.pi, 0, 0])
                link = self.openrave_robot.GetLink("right_HandFingerOneKnuckleThreeLink")
                T_E_L = T_E_W * self.OpenraveToKDL(link.GetTransform())
                TR_E_L = PyKDL.Frame(T_E_L.M)
                for pt_idx in sampled_points:
                    pt_L = surface_points[pt_idx]
                    pt_E = T_E_L * pt_L.pos
                    n_E = TR_E_L * pt_L.normal
                    points.append([0, pt_E, n_E, pt_idx, (sp_idx, f1_idx, None, None)])
                for pt_L in points_in_link:
                    pt_E = T_E_L * pt_L
                    points_forbidden.append([0, pt_E, None, None, (sp_idx, f1_idx, None, None)])

              for f2_idx in range(len(f2_configs)):
                f2 = f2_configs[f2_idx]
                self.openrave_robot.SetDOFValues([sp/180.0*math.pi, 0, 0, f2/180.0*math.pi])
                link = self.openrave_robot.GetLink("right_HandFingerTwoKnuckleThreeLink")
                T_E_L = T_E_W * self.OpenraveToKDL(link.GetTransform())
                TR_E_L = PyKDL.Frame(T_E_L.M)
                for pt_idx in sampled_points:
                    pt_L = surface_points[pt_idx]
                    pt_E = T_E_L * pt_L.pos
                    n_E = TR_E_L * pt_L.normal
                    points.append([1, pt_E, n_E, pt_idx, (sp_idx, None, None, f2_idx)])
                for pt_L in points_in_link:
                    pt_E = T_E_L * pt_L
                    points_forbidden.append([0, pt_E, None, None, (sp_idx, None, None, f2_idx)])

            for f3_idx in range(len(f3_configs)):
                f3 = f3_configs[f3_idx]
                self.openrave_robot.SetDOFValues([sp/180.0*math.pi, 0, f3/180.0*math.pi, 0])
                link = self.openrave_robot.GetLink("right_HandFingerThreeKnuckleThreeLink")
                T_E_L = T_E_W * self.OpenraveToKDL(link.GetTransform())
                TR_E_L = PyKDL.Frame(T_E_L.M)
                for pt_idx in sampled_points:
                    pt_L = surface_points[pt_idx]
                    pt_E = T_E_L * pt_L.pos
                    n_E = TR_E_L * pt_L.normal
                    points.append([2, pt_E, n_E, pt_idx, (None, None, f3_idx, None)])
                for pt_L in points_in_link:
                    pt_E = T_E_L * pt_L
                    points_forbidden.append([0, pt_E, None, None, (None, None, f3_idx, None)])

            #
            # check self collisions of the gripper
            #
            self_collision_set_filename = "self_collision.txt"
            self_collisions_configs = set()
            if False:
                print "checking self collision for %s configurations..."%(len(sp_configs) * len(f1_configs) * len(f2_configs) * len(f3_configs))
                self_collisions_configs = self.generateSelfCollisionData(sp_configs, f1_configs, f2_configs, f3_configs)
                print "done."
                self.saveSelfCollisionData(self_collision_set_filename, self_collisions_configs)
            else:
                self_collisions_configs = self.loadSelfCollisionData(self_collision_set_filename)

            print "points: %s"%(len(points))

            voxel_grid = volumetricutils.VoxelGrid(0.005)
            voxel_grid.build(points, points_forbidden)
            print "dim_min: %s"%(voxel_grid.dim_min)
            print "dim_max: %s"%(voxel_grid.dim_max)
            print "voxel_map_size: %s %s %s"%(voxel_grid.grid_size[0], voxel_grid.grid_size[1], voxel_grid.grid_size[2])
            print "voxels: %s"%(voxel_grid.grid_size[0]*voxel_grid.grid_size[1]*voxel_grid.grid_size[2])
            print "max_points_in_voxel: %s"%(voxel_grid.max_points_in_voxel)
            print "max_points_in_voxel (f): %s"%(voxel_grid.max_points_in_voxel_f)

            # move the sphere around
            sphere_diameter = 0.044
            sphere_radius = 0.5 * sphere_diameter

            cylinder_diameter = 0.0305
            cylinder_radius = 0.5 * sphere_diameter

            voxel = voxel_grid.grid[int(voxel_grid.grid_size[0]/2.0)][int(voxel_grid.grid_size[1]/2.0)][int(voxel_grid.grid_size[2]/2.0)]
            print "voxel: %s"%(len(voxel))

            # get the neighborhood
            init_pos = voxel[0][1]
            print "pos: %s"%(init_pos)

            offsets = []
            for x in np.linspace(-0.01, 0.01, 5):
              for y in np.linspace(-0.01, 0.01, 5):
                for z in np.linspace(-0.01, 0.01, 5):
                  offsets.append(PyKDL.Vector(x,y,z))

            print "number of candidate positions: %s"%(len(offsets))
            valid_positions = []

            print "searching for good position for grasping..."
            offset_idx = 0
            for offset in offsets:
              print "offset_idx: %s"%(offset_idx)
              offset_idx += 1
#            offset = PyKDL.Vector(0,0,0)
#            if True:
              pos = init_pos + offset

              points_in_sphere, points_f_in_sphere, valid_configurations, neighborhood_size = voxel_grid.getPointsAtPoint(pos, sphere_radius)

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
                  continue

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
                  continue

              # update the points set
              points_in_sphere2 = []
              for pt in points_in_sphere:
                  q = pt[4]
                  if q[0] == None or q[0] in spread_values:
                      points_in_sphere2.append(pt)

              points_in_sphere = points_in_sphere2

              points_f_in_sphere2 = []
              for pt in points_f_in_sphere:
                  q = pt[4]
                  if q[0] == None or q[0] in spread_values:
                      points_f_in_sphere2.append(pt)

              points_f_in_sphere = points_f_in_sphere2

              points_for_config = {}
              for pt in points_in_sphere:
                  if not pt[4] in points_for_config:
                      points_for_config[pt[4]] = [pt]
                  else:
                      points_for_config[pt[4]].append(pt)

              print "points_in_sphere: %s"%(len(points_in_sphere))
              print "configs: %s"%(len(points_for_config))

              points_f_for_config = {}
              for pt in points_f_in_sphere:
                  if not pt[4] in points_f_for_config:
                      points_f_for_config[pt[4]] = [pt]
                  else:
                      points_f_for_config[pt[4]].append(pt)

              print "points_f_in_sphere: %s"%(len(points_f_in_sphere))
              print "configs_f: %s"%(len(points_f_for_config))

              TT_E_H = PyKDL.Frame(pos)
              T_E_O_dict = {}
              TR_E_O_dict = {}

              normals_for_config = {}
              contacts_obj_for_config = {}
              contacts_normal_obj_for_config = {}
              contacts_link_for_config = {}
              is_plane_obj_config = {}
              ori_for_config = {}
              config_pt_ori = {}
              total_loops = 0
              print "calculating valid orientations..."
              # for each config calculate valid orientations
              points_for_config2 = {}
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

              print "done (%s)."%(total_loops)

              points_for_config = points_for_config2
#              print "configs: %s"%(len(points_for_config))

              points_for_config_f1 = {}
              points_for_config_f2 = {}
              points_for_config_f3 = {}

              for cf in points_for_config:
                  if cf[1] != None:
                      points_for_config_f1[cf] = points_for_config[cf]
                  elif cf[3] != None:
                      points_for_config_f2[cf] = points_for_config[cf]
                  elif cf[2] != None:
                      points_for_config_f3[cf] = points_for_config[cf]
                  else:
                      print "error: wrong hand configuration %s"%(offset)
                      exit(0)

              print "joining fingers configurations..."

              ori_for_config2 = {}
              points_for_config2 = {}

              # visualization
              if False:
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

                      self.pub_marker.eraseMarkers(0,6000, frame_id='world')
                      print "orientations: %s"%(len(ori_for_config[cf]))
                      m_id = 0
                      for ori in ori_for_config[cf]:
                          T_W_O = T_W_E * TT_E_H * orientations[ori] * T_H_O
                          m_id = self.pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)
                          rospy.sleep(0.001)

                      for ori in ori_for_config[cf]:
                          o_q = orientations[ori].M.GetQuaternion()
                          o_pt = PyKDL.Vector(o_q[0], o_q[1], o_q[2])
                          m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(0,0,0.6) + o_pt*0.1, m_id, r=0, g=o_q[3], b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                          rospy.sleep(0.001)

                      while True:
                          ch = raw_input("x")
                          if ch == 'n':
                              break
                          self.pub_marker.eraseMarkers(0,6000, frame_id='world')
                          ori = random.choice(tuple(ori_for_config[cf]))
                          T_W_O = T_W_E * TT_E_H * orientations[ori] * T_H_O
                          m_id = self.pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=0, g=1, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)

                          for pt in contacts_link_for_config[(cf,ori)]:
                              m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=0, g=0, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=T_W_E)

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
                      ori_set_ok = set()
                      for ori_idx in ori_set:

                          T_E_O = TT_E_H * orientations[ori_idx] * T_H_O
                          TR_E_O = PyKDL.Frame(T_E_O.M)

                          for cfx in [cf1, cf2, cf3]:
                              ori_ok = False
                              if ori_idx in forbidden_cf_ori[cfx]:
                                  break
                              if (cfx,ori_idx) in contacts_link_for_config:
                                  continue
                              for f_pt in points_for_config[cfx]:
                                  vol_idx = vol_obj.getVolIndex(f_pt[1]-pos)
                                  vol_sample = vol_obj.vol_samples[vol_idx[0]][vol_idx[1]][vol_idx[2]]
                                  if not ori_idx in vol_sample:
                                      continue

                                  norm, type_surf = vol_sample[ori_idx]
                                  normal_obj_E = TR_E_O * norm
                                  # check the contact between two surfaces
                                  if surface_points[f_pt[3]].is_plane and type_surf==0:
                                      if PyKDL.dot(normal_obj_E, f_pt[2]) < -0.8:
                                          ori_ok = True
                                  elif (surface_points[f_pt[3]].is_plane and type_surf==2) or (surface_points[f_pt[3]].is_point and type_surf==0):
                                      if PyKDL.dot(normal_obj_E, f_pt[2]) < -0.3:
                                          ori_ok = True
                                  elif (surface_points[f_pt[3]].is_plane and type_surf==1) or (surface_points[f_pt[3]].is_edge and type_surf==0):
                                      if PyKDL.dot(normal_obj_E, f_pt[2]) < -0.3:
                                          ori_ok = True
                                  elif surface_points[f_pt[3]].is_edge and type_surf==1:
                                      if PyKDL.dot(normal_obj_E, f_pt[2]) < -0.3:
                                          ori_ok = True

                                  if ori_ok:
                                      if not (cfx,ori_idx) in contacts_link_for_config:
                                          contacts_link_for_config[(cfx,ori_idx)] = [f_pt[1]]
                                      else:
                                          contacts_link_for_config[(cfx,ori_idx)].append(f_pt[1])
                                      if not (cfx,ori_idx) in contacts_obj_for_config:
                                          contacts_obj_for_config[(cfx,ori_idx)] = [pos + vol_obj.getVolPoint(vol_idx[0],vol_idx[1],vol_idx[2])]
                                      else:
                                          contacts_obj_for_config[(cfx,ori_idx)].append(pos + vol_obj.getVolPoint(vol_idx[0],vol_idx[1],vol_idx[2]))
                                      if not (cfx,ori_idx) in normals_for_config:
                                          normals_for_config[(cfx,ori_idx)] = [normal_obj_E]
                                      else:
                                          normals_for_config[(cfx,ori_idx)].append(normal_obj_E)
                                      is_plane_obj_config[(cfx,ori_idx)] = type_surf==0
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
                          for disabled_pt_idx in sampled_points2_obj:
                              pt_O = surface_points_obj[disabled_pt_idx].pos
                              pt_E = TT_E_H * orientations[ori_idx] * T_H_O * pt_O
                              xi, yi, zi = voxel_grid.getPointIndex(pt_E)
                              if xi >= voxel_grid.grid_size[0] or xi < 0 or yi >= voxel_grid.grid_size[1] or yi < 0 or zi >= voxel_grid.grid_size[2] or zi < 0:
                                  continue
                              for pt_gr in voxel_grid.grid[xi][yi][zi]:
                                  if pt_gr[4] == cf1 or pt_gr[4] == cf2 or pt_gr[4] == cf3:
                                      collision = True
                                      break

                          if not collision:
                              ori_set_ok.add(ori_idx)
                              contacts_obj_for_config[(cf,ori_idx)] = contacts_obj_for_config[(cf1,ori_idx)] + contacts_obj_for_config[(cf2,ori_idx)] + contacts_obj_for_config[(cf3,ori_idx)]
                              contacts_link_for_config[(cf,ori_idx)] = contacts_link_for_config[(cf1,ori_idx)] + contacts_link_for_config[(cf2,ori_idx)] + contacts_link_for_config[(cf3,ori_idx)]
                              normals_for_config[(cf,ori_idx)] = normals_for_config[(cf1,ori_idx)] + normals_for_config[(cf2,ori_idx)] + normals_for_config[(cf3,ori_idx)]

                      if len(ori_set_ok) > 0:
                          good_poses += len(ori_set_ok)
                          good_configs[(cf1[0], cf1[1], cf3[2], cf2[3])] = ori_set_ok
              print "done."

              print "all_configs: %s"%(all_configs)
              print "good_poses: %s"%(good_poses)
              print "good_configs: %s"%(len(good_configs))

              print "points_for_config: %s"%(len(points_for_config))
              total_points = 0
              for cf in points_for_config:
                  total_points += len(points_for_config[cf])
              print "points_for_config total_points: %s"%(total_points)

              total_points = 0
              for cf in ori_for_config:
                  total_points += len(ori_for_config[cf])
              print "points_for_config total_orientations: %s"%(total_points)

              contacts_cf_ori = {}
              wrenches_cf_ori = {}
              # get contact points for each good grasp
              for cf in good_configs:
                  ori_set = good_configs[cf]
                  for ori in ori_set:
                      contacts = []
                      for pt_idx in range(len(contacts_obj_for_config[(cf,ori)])):
                          pt_E = contacts_obj_for_config[(cf,ori)][pt_idx]
                          norm_E = normals_for_config[(cf,ori)][pt_idx]
                          similar = False
                          for c in contacts:
                              if (c[0]-pt_E).Norm() < 0.003 and velmautils.getAngle(c[1],norm_E) < 20.0/180.0*math.pi:
                                  similar = True
                                  break
                          if not similar:
                              contacts.append((pt_E, norm_E))
                      contacts_cf_ori[(cf, ori)] = contacts
                      wrenches_cf_ori[(cf, ori)] = []
                      for c in contacts:
                          wrenches_cf_ori[(cf, ori)] += self.contactToWrenches(c[0]-pos, c[1], 0, 1)

              # initial selection of grasps based on contact forces analysis
              print "initial selection of grasps based on contact forces analysis..."
              good_configs2 = {}
              contacts_cf_ori2 = {}
              wrenches_cf_ori2 = {}
              good_poses = 0
              for cf, ori in wrenches_cf_ori:
                  grip_ok = True
                  for n in normals_sphere_contacts:
                      min_f = None
                      max_f = None
                      min_t = None
                      max_t = None
                      for wr in wrenches_cf_ori[(cf, ori)]:
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
#                      print cf, ori, n, min_f, max_f, min_t, max_t
                      if max_f < 0.0 or min_f > 0.0 or max_t < 0.0 or min_t > 0.0:
                          grip_ok = False
                          break
                  if grip_ok:
                      if not cf in good_configs2:
                          good_configs2[cf] = set([ori])
                      else:
                          good_configs2[cf].add(ori)
                      contacts_cf_ori2[(cf,ori)] = contacts_cf_ori[(cf,ori)]
                      wrenches_cf_ori2[(cf,ori)] = wrenches_cf_ori[(cf,ori)]
                      good_poses += 1

              print "done."
              print "good_poses: %s"%(good_poses)
              print "good_configs: %s"%(len(good_configs))

              # visualization
              if False:
                  for cf in good_configs:
                      ori_set = good_configs[cf]
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
                      for ori in ori_set:
                         print cf, ori
                         self.pub_marker.eraseMarkers(0,2000, frame_id='world')
                         m_id = 0
                         T_W_O = T_W_E * TT_E_H * orientations[ori] * T_H_O
                         m_id = self.pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)
                         for c in contacts_cf_ori[(cf, ori)]:
                             m_id = self.pub_marker.publishVectorMarker(T_W_E*c[0], T_W_E*(c[0]+c[1]*0.004), m_id, r=1, g=1, b=1, namespace='default', frame='world', scale=0.0005)

                         for pt in contacts_link_for_config[(cf,ori)]:
                             m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=0, g=0, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=T_W_E)

                         raw_input("Press ENTER to continue...")


if __name__ == '__main__':

    rospy.init_node('grasp_leanring')

    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)

    task.spin()


