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

import random
import PyKDL
import math
import numpy as np
import copy
import scipy
import operator
import rospy
from geometry_msgs.msg import *
from visualization_msgs.msg import *

def pointInTriangle(A, B, C, P):
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

    if dot00 * dot11 - dot01 * dot01 == 0.0:
        return False
    # Compute barycentric coordinates
    invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * invDenom
    v = (dot00 * dot12 - dot01 * dot02) * invDenom

    # Check if point is in triangle
    return (u >= 0) and (v >= 0) and (u + v < 1)

def readMesh(filename):
    with open(filename, 'r') as f:
        header = f.readline()
        header_s = header.split()
        vert_count = int(header_s[0])
        face_count = int(header_s[1])
        vertices = []
        faces = []
        for i in range(vert_count):
            line = f.readline()
            line_s = line.split()
            vertices.append([float(line_s[0]),float(line_s[1]),float(line_s[2])])
        for i in range(face_count):
            line = f.readline()
            line_s = line.split()
            faces.append([int(line_s[0]),int(line_s[1]),int(line_s[2])])
    return vertices, faces

def readStl(filename, scale = 1.0):
    line_no = 0
    vertices_s = []
    faces = []
    with open(filename, 'r') as f:
        header = f.readline()
        line_no += 1
        eof = False
        while True:
            lines = []
            for i in range(7):
                line = f.readline()
                if line == None or line == '':
                    eof = True
                    break
                lines.append(line)
            if eof:
                break

            vertex = []
            vertex.append(lines[2].split())
            vertex.append(lines[3].split())
            vertex.append(lines[4].split())
            face = []
            for i in range(3):
                if vertex[i][0] != 'vertex':
                    print "readStl: error on line %s"%(line_no+2+i)
                found_vertex = False
                for v_id in range(len(vertices_s)):
                    if vertices_s[v_id][0] == vertex[i][1] and vertices_s[v_id][1] == vertex[i][2] and vertices_s[v_id][2] == vertex[i][3]:
                        found_vertex = True
                        break
                if not found_vertex:
                    vertices_s.append([vertex[i][1], vertex[i][2], vertex[i][3]])
                    v_id = len(vertices_s) - 1
                face.append(v_id)
            faces.append(face)
            line_no += 7

    vertices = []
    for v_s in vertices_s:
        vertices.append([scale*float(v_s[0]), scale*float(v_s[1]), scale*float(v_s[2])])

#facet normal -0.514031 0.017857 -0.857586
#outer loop
#vertex -26.348042 5.528012 -2.000000
#vertex -26.463844 2.194508 -2.000000
#vertex -28.000000 6.000000 -1.000000
#endloop
#endfacet

    return vertices, faces

class SurfacePoint:

    def __init__(self):
        self.id = -1
        self.pos = None
        self.normal = None
        self.frame = None
        self.pc1 = None
        self.pc2 = None
        self.neighbors_id = []
        self.visited = False
        self.allowed = True
        self.contact_regions = None

# this code is taken from PCL (https://github.com/PointCloudLibrary/pcl/blob/pcl-1.7.2/features/include/pcl/features/impl/principal_curvatures.hpp)
def pclPrincipalCurvaturesEstimation(surf_points, p_idx, neighborhood, neighborhood_dist):

    def getNeighborhood(surf_points, p_idx, depth):
        result = [p_idx]
        if depth == 0:
            return result
        for n_id in surf_points[p_idx].neighbors_id:
            if surf_points[n_id].visited:
                continue
            surf_points[n_id].visited = True
            neigh = getNeighborhood(surf_points, n_id, depth-1)
            for n in neigh:
                if not n in result:
                    result.append(n)
        return result

    for pt in surf_points:
        pt.visited = False

    surf_points[p_idx].visited = True
    indices_all = getNeighborhood(surf_points, p_idx, neighborhood)

    indices = []
    for idx in indices_all:
        if (surf_points[idx].pos-surf_points[p_idx].pos).Norm() <= neighborhood_dist:
            indices.append(idx)

    

    I = np.zeros((3,3))
    I[0,0] = 1
    I[1,1] = 1
    I[2,2] = 1

#    print I
    n_idx = np.array([surf_points[p_idx].normal[0], surf_points[p_idx].normal[1], surf_points[p_idx].normal[2]])
    n_idx2 = np.matrix(n_idx).transpose() * np.matrix(n_idx)
#    print n_idx2

    M = np.matrix(I - n_idx2)
#    print M


    # project the normals into the tangent plane
    projected_normals = []
    xyz_centroid = np.matrix([0.0, 0.0, 0.0]).transpose()
    for idx in indices:
        normal = np.matrix([surf_points[idx].normal[0], surf_points[idx].normal[1], surf_points[idx].normal[2]]).transpose()
#        print "n: %s"%(normal) 
        proj_normal = M * normal
#        print "pn: %s"%(proj_normal)
        projected_normals.append(proj_normal)
        xyz_centroid += proj_normal

    xyz_centroid = 1.0/float(len(indices)) * xyz_centroid

#    print "xyz_centroid: %s"%(xyz_centroid)

    covariance_matrix = np.matrix(np.zeros((3,3)))

    for n in projected_normals:
        demean_ = n - xyz_centroid;
        demean_xy = demean_[0] * demean_[1];
        demean_xz = demean_[0] * demean_[2];
        demean_yz = demean_[1] * demean_[2];
        covariance_matrix[0, 0] += demean_[0] * demean_[0];
        covariance_matrix[0, 1] += demean_xy;
        covariance_matrix[0, 2] += demean_xz;
        covariance_matrix[1, 0] += demean_xy;
        covariance_matrix[1, 1] += demean_[1] * demean_[1];
        covariance_matrix[1, 2] += demean_yz;
        covariance_matrix[2, 0] += demean_xz;
        covariance_matrix[2, 1] += demean_yz;
        covariance_matrix[2, 2] += demean_[2] * demean_[2];

#    print covariance_matrix

#    ei = scipy.sparse.linalg.eigsh(covariance_matrix,2)
#    print "scipy.sparse.linalg.eigsh"
#    print ei

    ei = np.linalg.eig(covariance_matrix)

    ei_sorted = sorted([[ei[0][0], 0], [ei[0][1], 1], [ei[0][2], 2]], key=operator.itemgetter(0))
#    print "np.linalg.eig"
#    print ei

#    print "ei_sorted: %s"%(ei_sorted)
    pcx = ei[1][0,ei_sorted[2][1]]
    pcy = ei[1][1,ei_sorted[2][1]]
    pcz = ei[1][2,ei_sorted[2][1]]

    pc1 = ei[0][ei_sorted[2][1]]
    pc2 = ei[0][ei_sorted[1][1]]

    pc = PyKDL.Vector(pcx, pcy, pcz)

    return indices, pc, pc1, pc2

def sampleMeshDetailedRandom(vertices, faces, sample_dist):
    point_id = 0
    points = []
#    face_start_point_id = []
    face_area = 0.0
    for face in faces:
#            face_start_point_id.append(point_id)
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

            # calculate the face normal
            normal = v0 * v1
            normal.Normalize()

            angle = getAngle(v0,v1)

            # calculate cumulated face area
            h_v0 = v1 * math.sin(angle)
            face_area += 0.5 * v0.Norm() * h_v0.Norm()

            while face_area > 0:
                r0 = 10.0
                r1 = 10.0
                while r0+r1 > 1.0:
                    r0 = random.random()
                    r1 = random.random()

                surf_pt = SurfacePoint()
                surf_pt.id = point_id
                surf_pt.pos = pt_a + v0*r0 + v1*r1
                surf_pt.normal = normal
                points.append(surf_pt)
                point_id += 1
                face_area -= sample_dist * sample_dist

            continue
            h = n1 * math.sin(angle)
            steps1 = int(h/sample_dist)
            if steps1 < 1:
                steps1 = 1
            step_len1 = h/steps1
            x0 = step_len0/2.0
            added = False
            while x0 < n0:
                x1 = step_len1/2.0
                while x1 < h*(1.0-x0/n0):
                    added = True
                    surf_pt = SurfacePoint()
                    surf_pt.id = point_id
                    surf_pt.pos = pt_a + v0*(x0/n0) + v1*(x1/h)
                    surf_pt.normal = normal
                    points.append(surf_pt)
                    point_id += 1
                    x1 += step_len1
                x0 += step_len0
            if not added:
                    print "adding %s"%( point_id)
                    surf_pt = SurfacePoint()
                    surf_pt.id = point_id
                    surf_pt.pos = (pt_a+pt_b+pt_c)/3.0
                    surf_pt.normal = normal
                    points.append(surf_pt)
                    point_id += 1
                
#    face_start_point_id.append(point_id)

####
    for pt_id in range(len(points)):
        for pt_id2 in range(len(points)):
            if pt_id2 == pt_id:
                continue
            dist = (points[pt_id].pos - points[pt_id2].pos).Norm()
            if dist < sample_dist*1.2:
                points[pt_id].neighbors_id.append(pt_id2)

    return points


def sampleMeshDetailed(vertices, faces, sample_dist):
    point_id = 0
    points = []
    face_start_point_id = []
    for face in faces:
            face_start_point_id.append(point_id)
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

            # calculate the face normal
            normal = v0 * v1
            normal.Normalize()

            angle = getAngle(v0,v1)
            h = n1*math.sin(angle)
            steps1 = int(h/sample_dist)
            if steps1 < 1:
                steps1 = 1
            step_len1 = h/steps1
            x0 = step_len0/2.0
            added = False
            while x0 < n0:
                x1 = step_len1/2.0
                while x1 < h*(1.0-x0/n0):
                    added = True
                    surf_pt = SurfacePoint()
                    surf_pt.id = point_id
                    surf_pt.pos = pt_a + v0*(x0/n0) + v1*(x1/h)
                    surf_pt.normal = normal
                    points.append(surf_pt)
                    point_id += 1
                    x1 += step_len1
                x0 += step_len0
            if not added:
                # calculate cumulated face area
                h_v0 = v1 * math.sin(angle)
                face_area = 0.5 * v0.Norm() * h_v0.Norm()
                if random.random()*sample_dist*sample_dist < face_area:
#                    print "adding %s"%( point_id)
                    surf_pt = SurfacePoint()
                    surf_pt.id = point_id
                    surf_pt.pos = (pt_a+pt_b+pt_c)/3.0
                    surf_pt.normal = normal
                    points.append(surf_pt)
                    point_id += 1
                
    face_start_point_id.append(point_id)

####
    for pt_id in range(len(points)):
        for pt_id2 in range(len(points)):
            if pt_id2 == pt_id:
                continue
            dist = (points[pt_id].pos - points[pt_id2].pos).Norm()
            if dist < sample_dist*1.2:
                points[pt_id].neighbors_id.append(pt_id2)

    return points
####
    for face_id in range(0, len(faces)):
        A_id = faces[face_id][0]
        B_id = faces[face_id][1]
        C_id = faces[face_id][2]
        # get id of all neigbouring faces and current face
        for face_id2 in range(0, len(faces)):
            if (A_id in faces[face_id2]) or (B_id in faces[face_id2]) or (C_id in faces[face_id2]):
                for pt_id in range(face_start_point_id[face_id], face_start_point_id[face_id+1]):
                    for pt_id2 in range(face_start_point_id[face_id2], face_start_point_id[face_id2+1]):
                        if pt_id2 == pt_id:
                            continue
                        dist = (points[pt_id].pos - points[pt_id2].pos).Norm()
                        if dist < sample_dist*2.0:
                            points[pt_id].neighbors_id.append(pt_id2)
        
    return points

def sampleMeshDetailedRays(vertices, faces, sample_dist):

    dim_min = [None, None, None]
    dim_max = [None, None, None]
    for v in vertices:
        for dim in range(3):
            if dim_min[dim] == None or dim_min[dim] > v[dim]:
                dim_min[dim] = v[dim]
            if dim_max[dim] == None or dim_max[dim] < v[dim]:
                dim_max[dim] = v[dim]

    rays_dim = [None, None, None]
    for dim in range(3):
        dim_min[dim] += 0.5 * sample_dist
#        rays_dim[i] = np.arange(dim_min[dim], dim_max[dim], sample_dist)

    other_dim = [[1,2],[0,2],[0,1]]

    point_id = 0
    points = []

    for face in faces:
        A = vertices[face[0]]
        B = vertices[face[1]]
        C = vertices[face[2]]
        pt_a = PyKDL.Vector(A[0],A[1],A[2])
        pt_b = PyKDL.Vector(B[0],B[1],B[2])
        pt_c = PyKDL.Vector(C[0],C[1],C[2])
        v0 = pt_b - pt_a
        v1 = pt_c - pt_a
        # calculate the face normal
        normal = v0 * v1
        normal.Normalize()
        # calculate the distance of the face from the origin
        fd = -PyKDL.dot(pt_a, normal)
        # get the direction (x,y,z) the face is pointing to
        normal_max = 0.0
        normal_max_dim = None
        for dim in range(3):
            if abs(normal[dim]) > normal_max:
                normal_max = abs(normal[dim])
                normal_max_dim = dim
        dimensions = other_dim[normal_max_dim]
        fdim_min = [None, None, None]
        fdim_max = [None, None, None]
        for pt in [pt_a, pt_b, pt_c]:
            for dim in range(3):
                if fdim_min[dim] == None or fdim_min[dim] > pt[dim]:
                    fdim_min[dim] = pt[dim]
                if fdim_max[dim] == None or fdim_max[dim] < pt[dim]:
                    fdim_max[dim] = pt[dim]

        n0_count = math.ceil((fdim_min[dimensions[0]] - dim_min[dimensions[0]])/sample_dist)
        n1_count = math.ceil((fdim_min[dimensions[1]] - dim_min[dimensions[1]])/sample_dist)

        for d0 in np.arange(dim_min[dimensions[0]] + sample_dist * n0_count, fdim_max[dimensions[0]], sample_dist):
            for d1 in np.arange(dim_min[dimensions[1]] + sample_dist * n1_count, fdim_max[dimensions[1]], sample_dist):
                pt_in = pointInTriangle([pt_a[dimensions[0]], pt_a[dimensions[1]]],
                [pt_b[dimensions[0]], pt_b[dimensions[1]]],
                [pt_c[dimensions[0]], pt_c[dimensions[1]]], [d0, d1])
                if pt_in:
                    d2 = -(normal[dimensions[0]]*d0 + normal[dimensions[1]]*d1 + fd)/normal[normal_max_dim]
                    pt_tuple = [None, None, None]
                    pt_tuple[dimensions[0]] = d0
                    pt_tuple[dimensions[1]] = d1
                    pt_tuple[normal_max_dim] = d2
                    surf_pt = SurfacePoint()
                    surf_pt.id = point_id
                    surf_pt.pos = PyKDL.Vector(pt_tuple[0], pt_tuple[1], pt_tuple[2])
                    surf_pt.normal = normal
                    points.append(surf_pt)
                    point_id += 1

    # put the points in voxels
    for dim in range(3):
        dim_min[dim] -= 0.5 * sample_dist
        dim_max[dim] += 1.5 * sample_dist

    def getPointIndex(pt):
        voxel_size = 10.0
        return [int((pt[0] - dim_min[0])/(sample_dist*voxel_size)), int((pt[1] - dim_min[1])/(sample_dist*voxel_size)), int((pt[2] - dim_min[2])/(sample_dist*voxel_size))]

    voxel_map_size = getPointIndex(dim_max)
    voxel_map_size[0] += 1
    voxel_map_size[1] += 1
    voxel_map_size[2] += 1

    voxel_map = []
    for x in range(voxel_map_size[0]):
        voxel_map.append([])
        for y in range(voxel_map_size[1]):
            voxel_map[-1].append([])
            for z in range(voxel_map_size[2]):
                voxel_map[-1][-1].append([])
                voxel_map[-1][-1][-1] = []

    # add all points to the voxel map
    for p in points:
        idx = getPointIndex(p.pos)
        voxel_map[idx[0]][idx[1]][idx[2]].append(p.id)

    def getSector(pt):
        x = pt[0]
        ax = abs(pt[0])
        y = pt[1]
        ay = abs(pt[1])
        z = pt[2]
        az = abs(pt[2])
        if ay <= x and az <= x:
            return 0
        elif ay <= -x and az <= -x:
            return 1
        elif az <= y and ax <= y:
            return 2
        elif az <= -y and ax <= -y:
            return 3
        elif ax <= z and ay <= z:
            return 4
        elif ax <= -z and ay <= -z:
            return 5

    # get neighbors
    for p in points:
        idx = getPointIndex(p.pos)
        idx_min = [
        idx[0]-1 if idx[0]>0 else idx[0],
        idx[1]-1 if idx[1]>0 else idx[1],
        idx[2]-1 if idx[2]>0 else idx[2]]
        idx_max = [
        idx[0]+1 if idx[0]<voxel_map_size[0] else idx[0],
        idx[1]+1 if idx[1]<voxel_map_size[1] else idx[1],
        idx[2]+1 if idx[2]<voxel_map_size[2] else idx[2]]

        ids = []
        for x in range(idx_min[0], idx_max[0]):
            for y in range(idx_min[1], idx_max[1]):
                for z in range(idx_min[2], idx_max[2]):
                    ids = ids + voxel_map[x][y][z]

        sectors = [None,None,None,None,None,None]    # +x -x +y -y +z -z

        for p_id in ids:
            diff = points[p_id].pos - p.pos
            sect = getSector(diff)
            dist = diff.Norm()
            if dist > sample_dist * 2.0:
                continue
            if sectors[sect] == None or sectors[sect][0] > dist:
                sectors[sect] = (dist, p_id)

        for s in sectors:
            if s == None:
                continue
            if not s[1] in p.neighbors_id:
                p.neighbors_id.append(s[1])
            if not p.id in points[s[1]].neighbors_id:
                points[s[1]].neighbors_id.append(p.id)

    return points

#
# Unit tests
#

def testSurfaceCurvature1(pub_marker, vertices, faces, T_W_O):
                print "generating surface..."
                surface_points = sampleMeshDetailedRays(vertices, faces, 0.001)
                points_neigh = []
                points_other = []
                print "calculating surface curvature..."
                curvatures = []
                for p_idx in range(len(surface_points)):
                    indices, nx, pc1, pc2 = pclPrincipalCurvaturesEstimation(surface_points, p_idx, 5, 0.003)
                    ny = surface_points[p_idx].normal * nx    # y = z * x
                    nx = ny * surface_points[p_idx].normal    # x = y * z
                    ny.Normalize()
                    nx.Normalize()
                    surface_points[p_idx].frame = PyKDL.Frame(PyKDL.Rotation(nx, ny, surface_points[p_idx].normal), surface_points[p_idx].pos)
                    surface_points[p_idx].pc1 = pc1
                    surface_points[p_idx].pc2 = pc2
                    if pc1 == 0:
                        curvatures.append(PyKDL.Vector(0, 0, 0.0))
                    else:
                        curvatures.append(PyKDL.Vector(pc1, pc2/pc1, 0.0))

                m_id = 0
                for pt in surface_points:
                    color = pt.pc1/10.0 + 0.5
                    if color > 1.0:
                        color = 1.0
                    if color < 0.0:
                        color = 0.0
                    color2 = pt.pc2/3.0 + 0.5
                    if color2 > 1.0:
                        color2 = 1.0
                    if color2 < 0.0:
                        color2 = 0.0
                    m_id = pub_marker.publishSinglePointMarker(pt.pos, m_id, r=color2, g=color, b=color, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.0005, 0.0005, 0.0005), T=T_W_O)
                    rospy.sleep(0.001)
                raw_input("Press ENTER to continue...")

def testSurfaceCurvature2(pub_marker, vertices, faces, T_W_O):

                print "generating surface..."
                surface_points = sampleMeshDetailedRays(vertices, faces, 0.001)
                points_neigh = []
                points_other = []
                p_idx = random.randint(0,len(surface_points)-1)
                print "p_idx: %s"%(p_idx)
                print "calculating surface curvature..."
                indices, nx, pc1, pc2 = pclPrincipalCurvaturesEstimation(surface_points, p_idx, 5, 0.003)

                print "pc: %s %s"%(pc1, pc2)

                for pt in surface_points:
                    if p_idx == pt.id:
                        continue
                    if pt.id in indices:
                        points_neigh.append(pt.pos)
                    else:
                        points_other.append(pt.pos)

                m_id = 0
                # publish the mesh of the object
                m_id = pub_marker.publishConstantMeshMarker("package://barrett_hand_defs/meshes/objects/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_O)

                m_id = pub_marker.publishMultiPointsMarker(points_other, m_id, r=0, g=0, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.0005, 0.0005, 0.0005), T=T_W_O)
                m_id = pub_marker.publishMultiPointsMarker(points_neigh, m_id, r=0, g=1, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.0005, 0.0005, 0.0005), T=T_W_O)
                m_id = pub_marker.publishSinglePointMarker(surface_points[p_idx].pos, m_id, r=0, g=1, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.0005, 0.0005, 0.0005), T=T_W_O)
                
                m_id = pub_marker.publishVectorMarker(T_W_O * surface_points[p_idx].pos, T_W_O * (surface_points[p_idx].pos + nx*0.03), m_id, 1, 0, 0, frame='world', namespace='default', scale=0.002)
                m_id = pub_marker.publishVectorMarker(T_W_O * surface_points[p_idx].pos, T_W_O * (surface_points[p_idx].pos + surface_points[p_idx].normal*0.03), m_id, 0, 0, 1, frame='world', namespace='default', scale=0.002)

                raw_input("Press ENTER to continue...")


