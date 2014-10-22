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
import thread
import velmautils
import itertools
import dijkstra

class MarkerLocalizator:
    """
Class for marker localization.
"""
    def spin(self):
        # locate all markers on the object
        if True:
            max_angle = 60.0/180.0*math.pi
            min_z = math.cos(max_angle)
            print "min_z: %s"%(min_z)

            # big_box
#            marker_list = [18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31]

            # small_box
#            marker_list = [32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45]

            # table
            marker_list = [3,4,5,6]
            mtrans_matrix = []
            for i in range(0, len(marker_list)):
                mtrans_matrix.append([])
                for j in range(0, len(marker_list)):
                    if i > j:
                        mtrans_matrix[i].append([])
                    else:
                        mtrans_matrix[i].append(None)

            marker_lock = Lock()
            self.visible_markers = []

            def markerCallback(data):
                marker_lock.acquire()
                self.visible_markers = []
                for m in data.markers:
                    if m.id in marker_list:
                        self.visible_markers.append( (m.id, pm.fromMsg(m.pose.pose)) )
                marker_lock.release()
                        
            rospy.Subscriber('/ar_pose_marker', AlvarMarkers, markerCallback)

            while not rospy.is_shutdown():
                marker_lock.acquire()
                visible_markers_local = copy.copy(self.visible_markers)
                marker_lock.release()

                accepted_markers = []
                accepted_markers_id = []
                for m in visible_markers_local:
                    n = PyKDL.Frame(m[1].M) * PyKDL.Vector(0,0,-1)
                    if n.z() > min_z:
                        accepted_markers.append( m )
                        accepted_markers_id.append( m[0] )

                print "visible: %s   accepted markers: %s"%(len(visible_markers_local), accepted_markers_id)
                for i in range(0, len(accepted_markers)):
                    for j in range(i+1, len(accepted_markers)):
                        if accepted_markers[i][0] > accepted_markers[j][0]:
                            idx1 = i
                            idx2 = j
                        else:
                            idx1 = j
                            idx2 = i
#                        print "   %s with %s"%(accepted_markers[idx1][0], accepted_markers[idx2][0])
                        T_C_M1 = accepted_markers[idx1][1]
                        T_C_M2 = accepted_markers[idx2][1]
                        T_M1_M2 = T_C_M1.Inverse() * T_C_M2
                        marker_id1_index = marker_list.index(accepted_markers[idx1][0])
                        marker_id2_index = marker_list.index(accepted_markers[idx2][0])
                        mtrans_matrix[marker_id1_index][marker_id2_index].append(T_M1_M2)
                rospy.sleep(0.1)

            G = {}
            for i in range(0, len(marker_list)):
                for j in range(0, len(marker_list)):
                    if mtrans_matrix[i][j] == None:
                        pass
#                        print "%s with %s:  None"%(marker_list[i], marker_list[j])
                    elif len(mtrans_matrix[i][j]) < 10:
                        print "%s with %s:  %s (ignoring)"%(marker_list[i], marker_list[j], len(mtrans_matrix[i][j]))
                    else:
                        score, R_M1_M2 = velmautils.meanOrientation(mtrans_matrix[i][j])
                        print "%s with %s:  %s, score: %s"%(marker_list[i], marker_list[j], len(mtrans_matrix[i][j]), score)
                        P_M1_M2 = velmautils.meanPosition(mtrans_matrix[i][j])
#                        P_M1_M2 = PyKDL.Vector()
#                        for k in range(0, len(mtrans_matrix[i][j])):
#                            P_M1_M2 += mtrans_matrix[i][j][k].p
#                        P_M1_M2 = P_M1_M2 * (1.0/len(mtrans_matrix[i][j]))
                        print "P_M1_M2: %s"%(P_M1_M2)
                        mtrans_matrix[i][j] = PyKDL.Frame(copy.deepcopy(R_M1_M2.M), P_M1_M2)
                        neighbours = G.get(marker_list[i], {})
                        if score > 0.01:
                            score = 1000000.0
                        else:
                            score = 1.0
                        neighbours[marker_list[j]] = score
                        G[marker_list[i]] = neighbours
                        neighbours = G.get(marker_list[j], {})
                        neighbours[marker_list[i]] = score
                        G[marker_list[j]] = neighbours

            frames = []
            # get the shortest paths weighted by score from optimization
            for marker_id in marker_list:
                path = dijkstra.shortestPath(G,marker_id,marker_list[0])
                print path
                T_Ml_Mf = PyKDL.Frame()
                for i in range(0, len(path)-1):
                    if marker_list.index(path[i]) > marker_list.index(path[i+1]):
                        T_Mp_Mn = mtrans_matrix[marker_list.index(path[i])][marker_list.index(path[i+1])]
                        T_Ml_Mf = T_Ml_Mf * T_Mp_Mn
                    else:
                        T_Mn_Mp = mtrans_matrix[marker_list.index(path[i+1])][marker_list.index(path[i])]
                        T_Ml_Mf = T_Ml_Mf * T_Mn_Mp.Inverse()

                frames.append(copy.deepcopy(T_Ml_Mf.Inverse()))

            marker_idx = 0
            for fr in frames:
                q = fr.M.GetQuaternion()
                p = fr.p
                print "[%s, PyKDL.Frame(PyKDL.Rotation.Quaternion(%s,%s,%s,%s),PyKDL.Vector(%s,%s,%s))],"%(marker_list[marker_idx], q[0], q[1], q[2], q[3], p.x(), p.y(), p.z())
                rospy.sleep(0.1)
                marker_idx += 1
            rospy.sleep(2.0)
            exit(0)

if __name__ == '__main__':

    rospy.init_node('locate_markers_on_object')
    task = MarkerLocalizator()
    task.spin()


