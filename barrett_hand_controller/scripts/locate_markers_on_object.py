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
from barrett_hand_controller_msgs.msg import *
from barrett_hand_controller_msgs.srv import *
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
import operator

class MarkerLocalizator:
    """
Class for marker localization.
"""
    def spin(self):
        # locate all markers on the object
        if False:
            max_angle = 60.0/180.0*math.pi
            min_z = math.cos(max_angle)
            print "min_z: %s"%(min_z)

            # big_box
#            marker_list = [18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31]

            # small_box
#            marker_list = [32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45]

            # table
#            marker_list = [3,4,5,6]

            # c-beam
            marker_list = [46, 47, 48, 49, 50, 51, 52, 53, 54, 55]
            mtrans_matrix = []
            mtrans_weights_ori = []
            mtrans_weights_pos = []
            for i in range(0, len(marker_list)):
                mtrans_matrix.append([])
                mtrans_weights_ori.append([])
                mtrans_weights_pos.append([])
                for j in range(0, len(marker_list)):
                    if i > j:
                        mtrans_matrix[i].append([])
                        mtrans_weights_ori[i].append([])
                        mtrans_weights_pos[i].append([])
                    else:
                        mtrans_matrix[i].append(None)
                        mtrans_weights_ori[i].append(None)
                        mtrans_weights_pos[i].append(None)

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
                accepted_markers_weights_pos = []
                accepted_markers_weights_ori = []
                for m in visible_markers_local:
                    n = PyKDL.Frame(m[1].M) * PyKDL.Vector(0,0,-1)
                    if n.z() > min_z:
                        accepted_markers.append( m )
                        accepted_markers_id.append( m[0] )

                        # n.z() is in range (min_z, 1.0), min_z is the best for orientation and 1.0 i best for position
                        weight_pos = (n.z() - min_z)/(1.0-min_z)
                        if weight_pos > 1.0 or weight_pos < 0.0:
                            print "error: weight==%s"%(weight_pos)

                        accepted_markers_weights_pos.append(weight_pos)
                        accepted_markers_weights_ori.append(1.0-weight_pos)

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

                        mtrans_weights_ori[marker_id1_index][marker_id2_index].append(accepted_markers_weights_ori[idx1] * accepted_markers_weights_ori[idx2])
                        mtrans_weights_pos[marker_id1_index][marker_id2_index].append(accepted_markers_weights_pos[idx1] * accepted_markers_weights_pos[idx2])

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
                        score, R_M1_M2 = velmautils.meanOrientation(mtrans_matrix[i][j], mtrans_weights_ori[i][j])
                        # ignore 20% the most distant measures from the mean
                        distances = []
                        for measure_idx in range(0, len(mtrans_matrix[i][j])):
                            diff = PyKDL.diff(R_M1_M2, mtrans_matrix[i][j][measure_idx]).rot.Norm()
                            distances.append([diff, measure_idx])
                        distances_sorted = sorted(distances, key=operator.itemgetter(0))
                        mtrans = []
                        weights_ori = []
                        for measure_idx in range(0, int(0.8*len(mtrans_matrix[i][j]))):
                            mtrans.append(mtrans_matrix[i][j][distances_sorted[measure_idx][1]])
                            weights_ori.append(mtrans_weights_ori[i][j][distances_sorted[measure_idx][1]])
#                        score, R_M1_M2 = velmautils.meanOrientation(mtrans, weights_ori)

                        print "%s with %s:  %s, score: %s"%(marker_list[i], marker_list[j], len(mtrans_matrix[i][j]), score)
                        P_M1_M2 = velmautils.meanPosition(mtrans_matrix[i][j], mtrans_weights_pos[i][j])
                        print "P_M1_M2 before: %s"%(P_M1_M2)
                        # ignore 20% the most distant measures from the mean
                        distances = []
                        for measure_idx in range(0, len(mtrans_matrix[i][j])):
                            diff = PyKDL.diff(R_M1_M2, mtrans_matrix[i][j][measure_idx]).vel.Norm()
                            distances.append([diff, measure_idx])
                        distances_sorted = sorted(distances, key=operator.itemgetter(0))
                        mtrans = []
                        weights_pos = []
                        for measure_idx in range(0, int(0.8*len(mtrans_matrix[i][j]))):
                            mtrans.append(mtrans_matrix[i][j][distances_sorted[measure_idx][1]])
                            weights_pos.append(mtrans_weights_ori[i][j][distances_sorted[measure_idx][1]])
#                        P_M1_M2 = velmautils.meanPosition(mtrans, weights_pos)

                        print "P_M1_M2 after: %s"%(P_M1_M2)
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

            pub_marker = velmautils.MarkerPublisher()
            rospy.sleep(1.0)
            m_id = 0
            marker_idx = 0
            for fr in frames:
                q = fr.M.GetQuaternion()
                p = fr.p
                print "[%s, PyKDL.Frame(PyKDL.Rotation.Quaternion(%s,%s,%s,%s),PyKDL.Vector(%s,%s,%s))],"%(marker_list[marker_idx], q[0], q[1], q[2], q[3], p.x(), p.y(), p.z())
                m_id = pub_marker.publishFrameMarker(fr, m_id, scale=0.1, frame='world', namespace='default')
                rospy.sleep(0.1)
                marker_idx += 1
            rospy.sleep(2.0)
            exit(0)
        else:
            pub_marker = velmautils.MarkerPublisher()
            rospy.sleep(1.0)
            markers2 = [
            [46, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1.0),PyKDL.Vector(-0.0,-0.0,-0.0))],
            [47, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.0161417497092,-9.92379339669e-05,-0.00603105214296,0.999851519216),PyKDL.Vector(-0.0987990318312,-0.000265582834399,0.000544628226204))],
            [48, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.706829255712,0.00114672638679,-0.707103949357,0.0198769487466),PyKDL.Vector(0.0427261427894,0.00245374838957,-0.116698579624))],
            [49, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.706230029879,-0.00660144281521,-0.70769079068,0.0192174565616),PyKDL.Vector(0.0410352237403,0.000595788239244,-0.0336956438972))],
            [50, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.0155276433453,0.714580229904,0.00743395758828,0.699341635824),PyKDL.Vector(-0.131991504795,-0.00410930997885,-0.0916799439467))],
            [51, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.706160148032,0.0114839861434,-0.707838133957,0.0130820300375),PyKDL.Vector(-0.150701418215,-0.000688426198715,-0.035762545196))],
            [52, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.705850422242,0.00260066465892,-0.708005925475,0.022271673869),PyKDL.Vector(-0.150663515172,-0.00196494029406,-0.123356224597))],
            [53, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.00630495805624,-0.999979097775,0.000308879730173,0.00139860956497),PyKDL.Vector(-0.0116053782242,0.000352840524022,-0.0267278839783))],
            [54, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.00853722085061,-0.999853611966,0.00230495916657,0.0146477869582),PyKDL.Vector(-0.0949889134574,0.00131718330416,-0.0165548984986))],
            [55, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0208301706621,-0.711731154203,0.0052945617051,0.702123091589),PyKDL.Vector(0.0244779541548,0.000463479220587,-0.0804749548707))],
            ]
            markers = [
            [46, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.0,1.0),PyKDL.Vector(-0.0,-0.0,-0.0))],
            [47, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.00211977224419,0.00133104595822,-0.00433238039783,0.999987482603),PyKDL.Vector(-0.0995553679408,-0.000651966932258,0.000444468330964))],
            [48, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.705539374795,0.00129956423061,-0.708394191186,0.0197527628665),PyKDL.Vector(0.0433256677966,0.00212664651843,-0.116482343501))],
            [49, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.704707757517,-0.00628228374428,-0.709255128279,0.0174548680028),PyKDL.Vector(0.0412709849952,0.000494665961165,-0.0338667341872))],
            [50, PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.0117276773848,0.706312439798,0.00558379104701,0.707781053891),PyKDL.Vector(-0.131246837996,-0.00469319026484,-0.0943814089463))],
            [51, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.710112841604,0.00963407546503,-0.703895468304,0.013345653983),PyKDL.Vector(-0.149984963191,-0.00300459973807,-0.0370193446712))],
            [52, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.704691425649,0.00497982940017,-0.709159595229,0.0218601100464),PyKDL.Vector(-0.15277553848,-0.00431480401095,-0.120995842686))],
            [53, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.00984066000086,-0.999945658681,0.00299307949816,0.00169781765667),PyKDL.Vector(-0.0132269965227,-0.00110379001368,-0.0274175040768))],
            [54, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.00154140261629,0.999633307109,-0.00751679824708,0.0259686953912),PyKDL.Vector(-0.0974091549673,-0.000670004842722,-0.0319416169884))],
            [55, PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0195903374145,-0.713404165076,0.00273342374532,0.700473585745),PyKDL.Vector(0.0250633176495,-0.00356911439713,-0.0811403242495))],
            ]
            m_id = 0
            for m in markers:
                print m[0]
                print m[1]
                m_id = pub_marker.publishFrameMarker(m[1], m_id, scale=0.1, frame='world', namespace='default')
                rospy.sleep(0.1)

if __name__ == '__main__':

    rospy.init_node('locate_markers_on_object')
    task = MarkerLocalizator()
    task.spin()


