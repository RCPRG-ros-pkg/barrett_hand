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
from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np
import copy
from scipy import optimize

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

class MarkerPublisher:
    def __init__(self):
        self.pub_marker = rospy.Publisher('/velma_markers', MarkerArray)

    def publishSinglePointMarker(self, pt, i, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005)):
        m = MarkerArray()
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = i
        marker.type = m_type
        marker.action = 0
        marker.pose = Pose( Point(pt.x(),pt.y(),pt.z()), Quaternion(0,0,0,1) )
        marker.scale = scale
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)
        self.pub_marker.publish(m)
        return i+1

    def publishMultiPointsMarker(self, pt, base_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=None):
        m = MarkerArray()
        ret_id = copy.copy(base_id)
        for i in range(0, len(pt)):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = ret_id
            ret_id += 1
            marker.type = m_type
            marker.action = 0
            if T != None:
                point = T*pt[i]
                marker.pose = Pose( Point(point.x(),point.y(),point.z()), Quaternion(0,0,0,1) )
            else:
                marker.pose = Pose( Point(pt[i].x(),pt[i].y(),pt[i].z()), Quaternion(0,0,0,1) )
            marker.scale = scale
            marker.color = ColorRGBA(r,g,b,0.5)
            m.markers.append(marker)
        self.pub_marker.publish(m)
        return ret_id

    def publishVectorMarker(self, v1, v2, i, r, g, b, frame='torso_base', namespace='default', scale=0.001):
        m = MarkerArray()
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = 0
        marker.points.append(Point(v1.x(), v1.y(), v1.z()))
        marker.points.append(Point(v2.x(), v2.y(), v2.z()))
        marker.pose = Pose( Point(0,0,0), Quaternion(0,0,0,1) )
        marker.scale = Vector3(scale, 2.0*scale, 0)
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)
        self.pub_marker.publish(m)

    def publishFrameMarker(self, T, base_id, scale=0.1, frame='torso_base', namespace='default'):
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(scale,0,0), base_id, 1, 0, 0, frame, namespace)
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(0,scale,0), base_id+1, 0, 1, 0, frame, namespace)
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(0,0,scale), base_id+2, 0, 0, 1, frame, namespace)
        return base_id+3

def getAngle(v1, v2):
    return math.atan2((v1*v2).Norm(), PyKDL.dot(v1,v2))

def generateNormalsSphere(angle):
    if angle <= 0:
        return None
    v_approach = []
    i = 0
    steps_alpha = int(math.pi/angle)
    if steps_alpha < 2:
        steps_alpha = 2
    for alpha in np.linspace(-90.0/180.0*math.pi, 90.0/180.0*math.pi, steps_alpha):
        max_steps_beta = (360.0/180.0*math.pi)/angle
        steps = int(math.cos(alpha)*max_steps_beta)
        if steps < 1:
            steps = 1
        beta_d = 360.0/180.0*math.pi/steps
        for beta in np.arange(0.0, 360.0/180.0*math.pi, beta_d):
            pt = PyKDL.Vector(math.cos(alpha)*math.cos(beta), math.cos(alpha)*math.sin(beta), math.sin(alpha))
            v_approach.append(pt)
            rospy.sleep(0.01)
    return v_approach

def generateFramesForNormals(angle, normals):
    steps = int((360.0/180.0*math.pi)/angle)
    if steps < 2:
        steps = 2
    angle_d = 360.0/180.0*math.pi/steps
    frames = []
    for z in normals:
        if PyKDL.dot(z, PyKDL.Vector(0,0,1)) > 0.9:
            y = PyKDL.Vector(1,0,0)
        else:
            y = PyKDL.Vector(0,0,1)
        x = y * z
        y = z * x
        for angle in np.arange(0.0, 359.9/180.0*math.pi, angle_d):
            frames.append(PyKDL.Frame(PyKDL.Rotation(x,y,z)) * PyKDL.Frame(PyKDL.Rotation.RotZ(angle)))
            print angle/math.pi*180.0

    return frames

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

    # Compute barycentric coordinates
    invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * invDenom
    v = (dot00 * dot12 - dot01 * dot02) * invDenom

    # Check if point is in triangle
    return (u >= 0) and (v >= 0) and (u + v < 1)

def sampleMesh(vertices, indices, sample_dist, pt_list, radius):
        points = []
        for face in indices:
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
            angle = getAngle(v0,v1)
            h = n1*math.sin(angle)
            steps1 = int(h/sample_dist)
            if steps1 < 1:
                steps1 = 1
            step_len1 = h/steps1
            x0 = step_len0/2.0
            while x0 < n0:
                x1 = step_len1/2.0
                while x1 < h*(1.0-x0/n0):
                    point = pt_a + v0*(x0/n0) + v1*(x1/h)
                    in_range = False
                    for s2 in pt_list:
                        if (point-s2).Norm() < radius:
                            in_range = True
                            break
                    if in_range:
                        points.append(point)
                    x1 += step_len1
                x0 += step_len0
        if len(pt_list) == 1:
            return points
        min_dists = []
        min_dists_p_index = []
        for s in pt_list:
            min_dists.append(1000000.0)
            min_dists_p_index.append(None)
        i = 0
        for s in pt_list:
            p_index = 0
            for p in points:
                d = (s-p).Norm()
                if d < min_dists[i]:
                    min_dists[i] = d
                    min_dists_p_index[i] = p_index
                p_index += 1
            i += 1
        first_contact_index = None
        for i in range(0, len(pt_list)):
            if min_dists[i] < sample_dist*2.0:
                first_contact_index = i
                break
        if first_contact_index == None:
            print "first_contact_index == None"
            return points
        init_pt = points[min_dists_p_index[first_contact_index]]
        points_ret = []
        list_to_check = []
        list_check_from = []
        for i in range(0, len(points)):
            if (init_pt-points[i]).Norm() > radius:
                continue
            if i == min_dists_p_index[first_contact_index]:
                list_check_from.append(points[i])
            else:
                list_to_check.append(points[i])
        points_ret = []
        added_point = True
        iteration = 0
        while added_point:
            added_point = False
            list_close = []
            list_far = []
            for p in list_to_check:
                added_p = False
                for check_from in list_check_from:
                    if (check_from-p).Norm() < sample_dist*2.0:
                        added_point = True
                        added_p = True
                        list_close.append(p)
                        break
                if not added_p:
                    list_far.append(p)
            points_ret += list_check_from
            list_to_check = copy.deepcopy(list_far)
            list_check_from = copy.deepcopy(list_close)
            iteration += 1
        return points_ret

def estPlane(points_in):
    mean_pt = PyKDL.Vector()
    for p in points_in:
        mean_pt += p
    mean_pt *= (1.0/len(points_in))

    points = []
    for p in points_in:
        points.append(p-mean_pt)

    def calc_R(xa, ya):
        ret = []
        """ calculate the minimum distance of each contact point from jar surface pt """
        n = PyKDL.Frame(PyKDL.Rotation.RotX(xa)) * PyKDL.Frame(PyKDL.Rotation.RotY(ya)) * PyKDL.Vector(0,0,1)
        for p in points:
            ret.append(PyKDL.dot(n,p))
        return numpy.array(ret)
        
    def f_2(c):
        """ calculate the algebraic distance between each contact point and jar surface pt """
        Di = calc_R(*c)
        return Di

    angles_estimate = 0.0, 0.0
    angles_2, ier = optimize.leastsq(f_2, angles_estimate, maxfev = 1000)
    n = PyKDL.Frame(PyKDL.Rotation.RotX(angles_2[0])) * PyKDL.Frame(PyKDL.Rotation.RotY(angles_2[1])) * PyKDL.Vector(0,0,1)

    nz = n
    if math.fabs(n.x()) < 0.9:
        nx = PyKDL.Vector(1,0,0)
    else:
        nx = PyKDL.Vector(0,1,0)

    ny = nz*nx
    nx = ny*nz
    nx.Normalize()
    ny.Normalize()
    nz.Normalize()

    return PyKDL.Frame(PyKDL.Rotation(nx,ny,nz), mean_pt)

def meanOrientation(T):
    R = []
    for t in T:
        R.append( copy.deepcopy( PyKDL.Frame(t.M) ) )

    def calc_R(rx, ry, rz):
        R_mean = PyKDL.Frame(PyKDL.Rotation.EulerZYX(rx, ry, rz))
        diff = []
        for r in R:
            diff.append(PyKDL.diff( R_mean, r ))
        ret = [math.fabs(d.rot.x()) for d in diff] + [math.fabs(d.rot.y()) for d in diff] + [math.fabs(d.rot.z()) for d in diff]
        return ret
    def f_2(c):
        """ calculate the algebraic distance between each contact point and jar surface pt """
        Di = calc_R(*c)
        return Di
    angle_estimate = R[0].M.GetEulerZYX()
    angle_2, ier = optimize.leastsq(f_2, angle_estimate, maxfev = 10000)
    score = calc_R(angle_2[0],angle_2[1],angle_2[2])
    score_v = 0.0
    for s in score:
        score_v += s*s
    return [score_v, PyKDL.Frame(PyKDL.Rotation.EulerZYX(angle_2[0],angle_2[1],angle_2[2]))]

# determine if a point is inside a given polygon or not
# Polygon is a list of (x,y) pairs.
def point_inside_polygon(x,y,poly):
    n = len(poly)
    inside =False
    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y
    return inside

class VelmaIkSolver:
    def __init__(self):
        pass

    def initIkSolver(self):
        self.robot = None
        try:
            self.robot = URDF.from_parameter_server()
        except:
            pass

        if self.robot == None:
            print "Could not load the robot description!"
            print "Please run <roscore> and then <roslaunch velma_description upload_robot.launch>"
            return

        print "len(self.robot.links) = %s"%(len(self.robot.links))
#        for l in self.robot.links:
#            print "name: %s"%(l.name)
#            print "visual:"
#            print l.visual
#            print "collision:"
#            print l.collision.geometry
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain("torso_link2", "right_HandPalmLink")

        self.q_min = PyKDL.JntArray(7)
        self.q_max = PyKDL.JntArray(7)
        self.q_limit = 0.26
        self.q_min[0] = -2.96 + self.q_limit
        self.q_min[1] = -2.09 + self.q_limit
        self.q_min[2] = -2.96 + self.q_limit
#        self.q_min[3] = -2.09 + self.q_limit
        self.q_min[3] = 0.1    # constraint on elbow
        self.q_min[4] = -2.96 + self.q_limit
        self.q_min[5] = -2.09 + self.q_limit
        self.q_min[6] = -2.96 + self.q_limit
#        self.q_max[0] = 2.96 - self.q_limit
        self.q_max[0] = 0.2    # constraint on first joint to avoid head hitting
        self.q_max[1] = 2.09 - self.q_limit
        self.q_max[2] = 2.96 - self.q_limit
        self.q_max[3] = 2.09 - self.q_limit
        self.q_max[4] = 2.96 - self.q_limit
        self.q_max[5] = 2.09 - self.q_limit
        self.q_max[6] = 2.96 - self.q_limit
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.vel_ik_solver = PyKDL.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_NR_JL(self.chain, self.q_min, self.q_max, self.fk_solver, self.vel_ik_solver, 100)
#        self.singularity_angle = 15.0/180.0*math.pi

    def simulateTrajectory(self, T_B_Einit, T_B_Ed, progress, q_start, T_T2_B):
        if progress < 0.0 or progress > 1.0:
            print "simulateTrajectory: bad progress value: %s"%(progress)
            return None

        q_end = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q_init = PyKDL.JntArray(7)
        for i in range(0,7):
            q_init[i] = q_start[i]

        for i in range(0, 7):
            if q_init[i] > self.q_max[i]-0.02:
                q_init[i] = self.q_max[i]-0.02
            if q_init[i] < self.q_min[i]+0.02:
                q_init[i] = self.q_min[i]+0.02

        q_out = PyKDL.JntArray(7)
        T_B_E_diff = PyKDL.diff(T_B_Einit, T_B_Ed, 1.0)
        T_B_Ei = PyKDL.addDelta(T_B_Einit, T_B_E_diff, progress)
        T_T2_Ei = T_T2_B * T_B_Ei
        status = self.ik_solver.CartToJnt(q_init, T_T2_Ei, q_out)
        if status != 0:
            return None, None
        for i in range(0, 7):
            q_end[i] = q_out[i]
        return q_end, T_B_Ei

    def getTrajCost(self, traj_T_B_Ed, q_start, T_T2_B, allow_q5_singularity_before_end, allow_q5_singularity_on_end):
        if len(traj_T_B_Ed) < 2:
            print "getTrajCost: wrong argument"
            return 1000000.0

#simulateTrajectory(T_B_Einit, T_B_Ed, progress, q_start, T_T2_B)

        q_init = copy.copy(q_start)
        for i in range(0, 7):
            if q_init[i] > self.q_max[i]-0.02:
                q_init[i] = self.q_max[i]-0.02
            if q_init[i] < self.q_min[i]+0.02:
                q_init[i] = self.q_min[i]+0.02

        q_out = PyKDL.JntArray(7)
        steps = 10
        time_set = np.linspace(1.0/steps, 1.0, steps)
        cost = 0.0
#        for T_B_Ed in traj_T_B_Ed:
        for i in range(0, len(traj_T_B_Ed)-1):

            for f in time_set:
                q_end, T_B_Ei = self.simulateTrajectory(traj_T_B_Ed[i], traj_T_B_Ed[i+1], f, q_init, T_T2_B)
                if q_end == None:
                    cost += 10000.0
                    return cost
#                T_B_Ei = PyKDL.addDelta(T_B_Eprev, T_B_E_diff, d)
#                T_T2_Ei = self.T_T2_B * T_B_Ei
#                status = self.ik_solver.CartToJnt(q_init, T_T2_Ei, q_out)
#                if status != 0:
#                    print "c"
#                    cost += 10000.0
#                    return cost
#                q5abs = math.fabs(q_out[5])
#                singularity = q5abs < self.abort_on_q5_singularity_angle + 5.0/180.0*math.pi
#                if allow_q5_singularity_before_end:
#                    pass
#                else:
#                    # punish for singularity
#                    if singularity:
#                        print "b"
#                        cost += 10000.0
#                        return cost
                for j in range(0, 7):
                    cost += (q_end[j] - q_init[j])*(q_end[j] - q_init[j])
                    q_init[j] = q_end[j]
#        if not allow_q5_singularity_on_end and singularity:
#            print "a"
#            cost += 10000.0
#            return cost

#        if q_end != None:
#            for i in range(0, 7):
#                q_end[i] = q_out[i]
        return cost

