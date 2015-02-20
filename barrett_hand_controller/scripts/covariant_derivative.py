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

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *

import PyKDL
import math
import numpy as np
import copy
import random
import velmautils
from subprocess import call
import operator

class MedialAxisTest:
    """
Class for medial axis computation test.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = pub_marker

    def spin(self):

        points = []
        dimx = 0.2
        dimy = 0.1
        dimz = 0.1

        vertices, faces = velmautils.readMesh('cube.mesh')

        vx = []
        for v in vertices:
            vx.append(PyKDL.Vector(dimx*v[0],dimy*v[1],dimz*v[2]))

        points, points_norm = velmautils.sampleMesh(vx, faces, 0.02, [PyKDL.Vector()], 10.0, return_normals=True)
        print "points: %s"%(len(points))

        variances = []
        for pt_i in range(len(points)):
            pt = points[pt_i]
            pt_norm = points_norm[pt_i]
            neigh, neigh_norm = velmautils.sampleMesh(vx, faces, 0.01, [pt], 0.02, return_normals=True)
            force_pt = pt_norm
            torque_pt = pt * pt_norm

            max_fd = None
            max_td = None
            for i in range(len(neigh)):
                force = neigh_norm[i]
                torque = neigh[i] * neigh_norm[i]

                force_dist = (force-force_pt).Norm()
                torque_dist = (torque-torque_pt).Norm()
                if max_fd == None or max_fd < force_dist:
                    max_fd = force_dist
                if max_td == None or max_td < torque_dist:
                    max_td = torque_dist

            variances.append(max_fd + max_td)
#            print max_fd, max_td
#            print len(neigh)

        m_id = 0
        for pt_i in range(len(points)):
            p = points[pt_i]
            sc = variances[pt_i]*0.02 + 0.005
            m_id = self.pub_marker.publishSinglePointMarker(p, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(sc,sc,sc), T=None)
            if pt_i%10 == 0:
                rospy.sleep(0.01)

#        m_id = 0
#        m_id = self.pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.005, 0.005, 0.005), T=None)
        rospy.sleep(1)

        exit(0)


        step = 0.02
        for x in np.arange(-dimx, dimx, step):
            for y in np.arange(-dimy, dimy, step):
                points.append(PyKDL.Vector(x,y,dimz))
                points.append(PyKDL.Vector(x,y,-dimz))
        for y in np.arange(-dimy, dimy, step):
            for z in np.arange(-dimz, dimz, step):
                points.append(PyKDL.Vector(dimx,y,z))
                points.append(PyKDL.Vector(-dimx,y,z))
        for z in np.arange(-dimz, dimz, step):
            for x in np.arange(-dimx, dimx, step):
                points.append(PyKDL.Vector(x,dimy,z))
                points.append(PyKDL.Vector(x,-dimy,z))

        if True:
            for i in range(len(points)):
                x = points[i].x()
                y = points[i].y()
                z = points[i].z()
                points[i] = PyKDL.Vector(x,y,z+x*x)

        print len(points)

        # get n_closest closest points around each point
        n_closest = 10
        closest_list = []
        for i in range(len(points)):
            closest = []
            for j in range(n_closest):
                closest.append([1000000.0,None])

            for j in range(len(points)):
                dist = (points[i]-points[j]).Norm()
                if closest[-1][1] == None or closest[-1][0] > dist:
                    closest[-1] = [dist, j]
                    sorted(closest, key=operator.itemgetter(0))
            closest_list.append(copy.deepcopy(closest))

        colors = []
        for i in range(len(points)):
            points[i]
            for pt_cl in closest_list[i]:
                pt = points[pt_cl[1]]


#        print len(points_medial)
#        m_id = 0
#        for p in points:
#            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(p[0], p[1], p[2]), m_id, r=0, g=1, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=None)

#        m_id = 0
#        for p in points_medial:
#            m_id = self.pub_marker.publishSinglePointMarker(p[0], m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(p[1],p[1],p[1]), T=None)
#        m_id = 0
#        m_id = self.pub_marker.publishMultiPointsMarkerWithSize(points_medial, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.SPHERE, T=None)
#        m_id = self.pub_marker.publishMultiPointsMarker(points_medial, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.002, 0.002, 0.002), T=None)
        rospy.sleep(1)

        

        exit(0)

if __name__ == '__main__':

    rospy.init_node('MedialAxisTest')

    pub_marker = velmautils.MarkerPublisher()
    task = MedialAxisTest(pub_marker)
    rospy.sleep(1)

    task.spin()


