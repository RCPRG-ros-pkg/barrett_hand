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

class VoxelGrid:

    def getPointIndex(self, pt):
        return (int((pt[0] - self.dim_min[0])/(self.voxel_size)), int((pt[1] - self.dim_min[1])/(self.voxel_size)), int((pt[2] - self.dim_min[2])/(self.voxel_size)))

    def __init__(self, voxel_size):
        self.voxel_size = voxel_size
        self.voxel_max_radius = math.sqrt(3)/2.0 * self.voxel_size
        self.dim_min = [None, None, None]
        self.dim_max = [None, None, None]

    def build(self, points, points_forbidden):
        for pt in points + points_forbidden:
            v = pt[1]
            for dim in range(3):
                if self.dim_min[dim] == None or self.dim_min[dim] > v[dim]:
                    self.dim_min[dim] = v[dim]
                if self.dim_max[dim] == None or self.dim_max[dim] < v[dim]:
                    self.dim_max[dim] = v[dim]

        self.grid_size = self.getPointIndex(self.dim_max)
        self.grid_size = (self.grid_size[0] + 1, self.grid_size[1] + 1, self.grid_size[2] + 1)

        self.grid = []
        self.grid_f = []
        for x in range(self.grid_size[0]):
            self.grid.append([])
            self.grid_f.append([])
            for y in range(self.grid_size[1]):
                self.grid[-1].append([])
                self.grid_f[-1].append([])
                for z in range(self.grid_size[2]):
                    self.grid[-1][-1].append([])
                    self.grid_f[-1][-1].append([])
                    self.grid[-1][-1][-1] = []
                    self.grid_f[-1][-1][-1] = []

        # add all points to the voxel map
        self.max_points_in_voxel = 0
        for p in points:
            idx = self.getPointIndex(p[1])
            self.grid[idx[0]][idx[1]][idx[2]].append(p)
            voxel_points_count = len(self.grid[idx[0]][idx[1]][idx[2]])
            if voxel_points_count > self.max_points_in_voxel:
                self.max_points_in_voxel = voxel_points_count

        self.max_points_in_voxel_f = 0
        for p in points_forbidden:
            idx = self.getPointIndex(p[1])
            self.grid_f[idx[0]][idx[1]][idx[2]].append(p)
            voxel_points_count = len(self.grid_f[idx[0]][idx[1]][idx[2]])
            if voxel_points_count > self.max_points_in_voxel_f:
                self.max_points_in_voxel_f = voxel_points_count

    def getPointsAtPoint(self, pos, radius):
        min_index = self.getPointIndex(pos - PyKDL.Vector(radius, radius, radius))
        max_index = self.getPointIndex(pos + PyKDL.Vector(radius, radius, radius))
        for dof in range(3):
            if min_index[dof] < 0:
                min_index[dof] = 0
            if max_index[dof] >= self.grid_size[dof]:
                max_index[dof] = self.grid_size[dof]-1

        # get the indices of voxels around the current point
        voxel_indices = []
        for x in range(min_index[0], max_index[0]+1):
            for y in range(min_index[1], max_index[1]+1):
                for z in range(min_index[2], max_index[2]+1):
                    voxel_center = PyKDL.Vector((x+0.5) * self.voxel_size + self.dim_min[0], (y+0.5) * self.voxel_size + self.dim_min[1], (z+0.5) * self.voxel_size + self.dim_min[2])
                    if (voxel_center-pos).Norm() > self.voxel_max_radius + radius:
                        continue
                    voxel_indices.append((x,y,z))

        points_in_sphere = []
        points_f_in_sphere = []
        valid_configurations = [[], [], []]
        for idx in voxel_indices:
            x,y,z = idx
            for pt in self.grid[x][y][z]:
                pt_diff = pt[1]-pos
                dist = pt_diff.Norm()
                if dist < radius:
                    q = pt[4]
                    if not q in valid_configurations[pt[0]]:
                        valid_configurations[pt[0]].append(q)
                    points_in_sphere.append(pt)

            for pt in self.grid_f[x][y][z]:
                pt_diff = pt[1]-pos
                dist = pt_diff.Norm()
                if dist < radius:
                    q = pt[4]
                    if not q in valid_configurations[pt[0]]:
                        valid_configurations[pt[0]].append(q)
                    points_f_in_sphere.append(pt)

        return points_in_sphere, points_f_in_sphere, valid_configurations

class VolumetricModel:
    def __init__(self, vol_radius, vol_samples_count, T_H_O, orientations_angle, vertices_obj, faces_obj):
        self.vol_radius = vol_radius
        self.vol_samples_count = vol_samples_count
        self.index_factor = float(self.vol_samples_count)/(2.0*self.vol_radius)
        self.vol_samples = []
        for x in np.linspace(-self.vol_radius, self.vol_radius, self.vol_samples_count):
            self.vol_samples.append([])
            for y in np.linspace(-self.vol_radius, self.vol_radius, self.vol_samples_count):
                self.vol_samples[-1].append([])
                for z in np.linspace(-self.vol_radius, self.vol_radius, self.vol_samples_count):
                    self.vol_samples[-1][-1].append([])
                    self.vol_samples[-1][-1][-1] = {}
        self.vol_sample_points = []
        for xi in range(self.vol_samples_count):
            for yi in range(self.vol_samples_count):
                for zi in range(self.vol_samples_count):
                    self.vol_sample_points.append( self.getVolPoint(xi,yi,zi) )
        self.T_H_O = T_H_O
        self.T_O_H = self.T_H_O.Inverse()

        # generate the set of orientations
        self.orientations_angle = orientations_angle
        normals_sphere = velmautils.generateNormalsSphere(self.orientations_angle)

        orientations1 = velmautils.generateFramesForNormals(self.orientations_angle, normals_sphere)
        orientations2 = []
        for ori in orientations1:
            x_axis = ori * PyKDL.Vector(1,0,0)
            if x_axis.z() > 0.0:
                orientations2.append(ori)
        self.orientations = {}
        for ori_idx in range(len(orientations2)):
            self.orientations[ori_idx] = orientations2[ori_idx]

        # generate a set of surface points
        self.surface_points_obj = surfaceutils.sampleMeshDetailedRays(vertices_obj, faces_obj, 0.0015)
        print "surface of the object has %s points"%(len(self.surface_points_obj))

        # disallow contact with the surface points beyond the key handle
        for p in self.surface_points_obj:
            if p.pos.x() > 0.0:
                p.allowed = False

        surface_points_obj_init = []
        surface_points2_obj_init = []
        for sp in self.surface_points_obj:
            if sp.allowed:
                surface_points_obj_init.append(sp)
            else:
                surface_points2_obj_init.append(sp)

        print "generating a subset of surface points of the object..."

        while True:
            p_idx = random.randint(0, len(self.surface_points_obj)-1)
            if self.surface_points_obj[p_idx].allowed:
                break
        p_dist = 0.003

        self.sampled_points_obj = []
        while True:
            self.sampled_points_obj.append(p_idx)
            surface_points_obj2 = []
            for sp in surface_points_obj_init:
                if (sp.pos-self.surface_points_obj[p_idx].pos).Norm() > p_dist:
                    surface_points_obj2.append(sp)
            if len(surface_points_obj2) == 0:
                break
            surface_points_obj_init = surface_points_obj2
            p_idx = surface_points_obj_init[0].id

        print "subset size: %s"%(len(self.sampled_points_obj))

        print "generating a subset of other surface points of the object..."

        p_dist2 = 0.006

        while True:
            p_idx = random.randint(0, len(self.surface_points_obj)-1)
            if not self.surface_points_obj[p_idx].allowed:
                break

        self.sampled_points2_obj = []
        while True:
            self.sampled_points2_obj.append(p_idx)
            surface_points2_obj2 = []
            for sp in surface_points2_obj_init:
                if (sp.pos-self.surface_points_obj[p_idx].pos).Norm() > p_dist2:
                    surface_points2_obj2.append(sp)
            if len(surface_points2_obj2) == 0:
                break
            surface_points2_obj_init = surface_points2_obj2
            p_idx = surface_points2_obj_init[0].id

        # test volumetric model
        if False:
            for pt_idx in self.sampled_points2_obj:
                pt = self.surface_points_obj[pt_idx]
                m_id = self.pub_marker.publishSinglePointMarker(pt.pos, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                rospy.sleep(0.001)

            for pt_idx in self.sampled_points_obj:
                pt = self.surface_points_obj[pt_idx]
                m_id = self.pub_marker.publishSinglePointMarker(pt.pos, m_id, r=0, g=1, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                rospy.sleep(0.001)

        print "subset size: %s"%(len(self.sampled_points2_obj))

        print "calculating surface curvature at sampled points of the obj..."
        m_id = 0
        planes = 0
        edges = 0
        points = 0
        for pt_idx in range(len(self.surface_points_obj)):
            indices, nx, pc1, pc2 = surfaceutils.pclPrincipalCurvaturesEstimation(self.surface_points_obj, pt_idx, 5, 0.003)
#            m_id = self.pub_marker.publishVectorMarker(self.T_W_O * self.surface_points_obj[pt_idx].pos, self.T_W_O * (self.surface_points_obj[pt_idx].pos + nx*0.004), m_id, 1, 0, 0, frame='world', namespace='default', scale=0.0002)
#            m_id = self.pub_marker.publishVectorMarker(self.T_W_O * self.surface_points_obj[pt_idx].pos, self.T_W_O * (self.surface_points_obj[pt_idx].pos + self.surface_points_obj[pt_idx].normal*0.004), m_id, 0, 0, 1, frame='world', namespace='default', scale=0.0002)
            self.surface_points_obj[pt_idx].frame = PyKDL.Frame(PyKDL.Rotation(nx, self.surface_points_obj[pt_idx].normal * nx, self.surface_points_obj[pt_idx].normal), self.surface_points_obj[pt_idx].pos)
            self.surface_points_obj[pt_idx].pc1 = pc1
            self.surface_points_obj[pt_idx].pc2 = pc2
            if pc1 < 0.2:
                self.surface_points_obj[pt_idx].is_plane = True
                self.surface_points_obj[pt_idx].is_edge = False
                self.surface_points_obj[pt_idx].is_point = False
                planes += 1
            elif pc2 < 0.2:
                self.surface_points_obj[pt_idx].is_plane = False
                self.surface_points_obj[pt_idx].is_edge = True
                self.surface_points_obj[pt_idx].is_point = False
                edges += 1
            else:
                self.surface_points_obj[pt_idx].is_plane = False
                self.surface_points_obj[pt_idx].is_edge = False
                self.surface_points_obj[pt_idx].is_point = True
                points += 1

        print "obj planes: %s  edges: %s  points: %s"%(planes, edges, points)











    def getVolIndex(self, pt):
        xi = int(np.floor( self.index_factor*(pt[0]+self.vol_radius) ))
        yi = int(np.floor( self.index_factor*(pt[1]+self.vol_radius) ))
        zi = int(np.floor( self.index_factor*(pt[2]+self.vol_radius) ))
        if xi < 0 or xi >= self.vol_samples_count or yi < 0 or yi >= self.vol_samples_count or zi < 0 or zi >= self.vol_samples_count:
            print "getVolIndex: error: %s, %s, %s"%(pt[0],pt[1],pt[2])
            return None
        return (xi, yi, zi)

    def getVolPoint(self, xi,yi,zi):
        return PyKDL.Vector(-self.vol_radius + (xi+0.5) / self.index_factor, -self.vol_radius + (yi+0.5) / self.index_factor, -self.vol_radius + (zi+0.5) / self.index_factor)

    def generate(self):

        for ori_idx in range(len(orientations)):
                    T_H_Hd = orientations[ori_idx]
                    T_H_Od = T_H_Hd * self.T_H_O
                    for surf_pt_idx in range(len(self.surface_points_obj)):
                        surf_pt = self.surface_points_obj[surf_pt_idx]
                        if not surf_pt.allowed:
                            continue
                        pt = T_H_Od * surf_pt.pos
                        vol_idx = self.getVolIndex(pt)
                        if vol_idx != None:
                            if not ori_idx in self.vol_samples[vol_idx[0]][vol_idx[1]][vol_idx[2]]:
                                self.vol_samples[vol_idx[0]][vol_idx[1]][vol_idx[2]][ori_idx] = [surf_pt.id]
                            else:
                                self.vol_samples[vol_idx[0]][vol_idx[1]][vol_idx[2]][ori_idx].append(surf_pt.id)
        print "transforming the volumetric map..."

        for xi in range(self.vol_samples_count):
                  print xi
                  for yi in range(self.vol_samples_count):
                    for zi in range(self.vol_samples_count):
                      for ori in self.vol_samples[xi][yi][zi]:
                          planes = 0
                          edges = 0
                          points = 0
                          norm = PyKDL.Vector()
                          for pt_id in self.vol_samples[xi][yi][zi][ori]:
                              norm += self.surface_points_obj[pt_id].normal
                              if self.surface_points_obj[pt_id].is_plane:
                                  planes += 1
                              if self.surface_points_obj[pt_id].is_edge:
                                  edges += 1
                              if self.surface_points_obj[pt_id].is_point:
                                  points += 1
                          norm.Normalize()
                          if planes >= edges and planes >= points:
                              self.vol_samples[xi][yi][zi][ori] = (norm, 0)
                          elif edges >= planes and edges >= points:
                              self.vol_samples[xi][yi][zi][ori] = (norm, 1)
                          else:
                              self.vol_samples[xi][yi][zi][ori] = (norm, 2)

    def save(self, filename):
        print "saving the volumetric map to file %s"%(vol_map_filename)
        with open(filename, 'w') as f:
                    f.write(str(self.vol_radius) + " " + str(self.vol_samples_count) + "\n")
                    for xi in range(self.vol_samples_count):
                      for yi in range(self.vol_samples_count):
                        for zi in range(self.vol_samples_count):
                            if len(self.vol_samples[xi][yi][zi]) > 0:
                                f.write(str(xi) + " " + str(yi) + " " + str(zi))
                                for ori_idx in self.vol_samples[xi][yi][zi]:
                                    norm, type_surf = self.vol_samples[xi][yi][zi][ori_idx]
                                    f.write(" " + str(ori_idx) + " " + str(norm[0]) + " " + str(norm[1]) + " " + str(norm[2]) + " " + str(type_surf))
                                f.write("\n")

    def load(self, filename):
        with open(filename, 'r') as f:
                    line = f.readline()
                    vol_radius_str, vol_samples_count_str = line.split()
                    vol_radius = float(vol_radius_str)
                    if vol_radius != self.vol_radius:
                        print "error: VolumetricModel.load: vol_radius != self.vol_radius"
                        return
                    vol_samples_count = int(vol_samples_count_str)
                    if vol_samples_count != self.vol_samples_count:
                        print "error: VolumetricModel.load: vol_samples_count != self.vol_samples_count"
                        return
                    while True:
                        line = f.readline()
                        val_str = line.split()
                        if len(val_str) == 0:
                            break
                        xi = int(val_str[0])
                        yi = int(val_str[1])
                        zi = int(val_str[2])
                        for i in range(3, len(val_str), 5):
                            ori_idx = int(val_str[i])
                            normx = float(val_str[i+1])
                            normy = float(val_str[i+2])
                            normz = float(val_str[i+3])
                            type_surf = int(val_str[i+4])
                            self.vol_samples[xi][yi][zi][ori_idx] = (PyKDL.Vector(normx, normy, normz), type_surf)

    def test1(self, pub_marker, T_W_H):
        scale = 2.0*self.vol_radius/self.vol_samples_count
        for ori_idx in range(len(self.orientations)):
            pub_marker.eraseMarkers(0, 1000, frame_id='world')
            m_id = 0
            T_W_O = T_W_H * self.orientations[ori_idx] * self.T_H_O
            m_id = pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)
            for pt in self.vol_sample_points:
                vol_idx = self.getVolIndex(pt)
                if vol_idx != None and ori_idx in self.vol_samples[vol_idx[0]][vol_idx[1]][vol_idx[2]]:
                    m_id = pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=1, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(scale, scale, scale), T=T_W_H)
                    rospy.sleep(0.001)
            raw_input("Press ENTER to continue...")

