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
import PyKDL
import math
import numpy as np
import copy
from scipy import optimize
import velmautils
import itertools
import dijkstra
import operator

class GraspableObject:
    def __init__(self, name, obj_type, size):
        self.name = name
        self.obj_type = obj_type
        self.size = size
        self.com = PyKDL.Vector()
        self.markers = []
        self.T_Br_Co = PyKDL.Frame()
        self.pose_update_time = rospy.Time.now()

    def addMarker(self, marker_id, T_Co_M):
        self.markers.append( (marker_id, T_Co_M) )

    def isBox(self):
        if self.obj_type == "box":
            return True
        return False

    def isCBeam(self):
        if self.obj_type == "cbeam":
            return True
        return False

    def updatePose(self, T_Br_Co):
        self.T_Br_Co = T_Br_Co
        self.pose_update_time = rospy.Time.now()

    def addComPoints(self, com_pt):
        self.com_pt = com_pt
        self.com_weights = list(np.zeros(len(self.com_pt)))

    def updateCom(self, T_B_O, T_B_O_2, contacts_O, m_id=0, pub_marker=None):
        ret_m_id = velmautils.updateCom(T_B_O, T_B_O_2, contacts_O, self.com_pt, self.com_weights, m_id=m_id, pub_marker=pub_marker)
        max_com = None
        for com_value in self.com_weights:
            if max_com == None or max_com < com_value:
                max_com = com_value
        mean_com = PyKDL.Vector()
        mean_count = 0
        for com_idx in range(0, len(self.com_weights)):
            com_value = self.com_weights[com_idx]
            if com_value == max_com:
                mean_com += self.com_pt[com_idx]
                mean_count += 1
        self.com = (1.0/float(mean_count)) * mean_com
        return ret_m_id

class Grip:
    def __init__(self, grasped_object):
        self.grasped_object = grasped_object
        self.contacts = []
        self.successful = False
        self.count_planning_failure = 0
        self.planning_failure_poses = []
        self.count_no_contact = 0
        self.count_too_little_contacts = 0
        self.count_moved_on_grip = 0
        self.count_unstable = 0
        self.count_stable = 0
        self.count_visibility_problem = 0
        self.visibility_problem_poses = []

    def setPlanningFailure(self, T_Br_O):
         self.count_planning_failure += 1
         self.planning_failure_poses.append(T_Br_O)

    def setNoContact(self):
         self.count_no_contact += 1

    def setTooLittleContacts(self):
         self.count_too_little_contacts += 1

    def setMovedOnGrip(self):
         self.count_moved_on_grip += 1

    def setUnstable(self):
        self.count_unstable += 1

    def setStable(self):
        self.count_stable += 1

    def setVisibilityProblem(self, T_Br_O):
        self.count_visibility_problem = 0
        self.visibility_problem_poses.append(T_Br_O)

    def addContact(self, T_O_Co):
        self.contacts.append(copy.deepcopy(T_O_Co))

    def success(self):
        self.successful = True

    def serializePrint(self):
        print "grips_db.append( Grip(obj_grasp) )"
        for c in self.contacts:
            q = c.M.GetQuaternion()
            print "grips_db[-1].addContact( PyKDL.Frame(PyKDL.Rotation.Quaternion(%s,%s,%s,%s),PyKDL.Vector(%s,%s,%s)) )"%(q[0], q[1], q[2], q[3], c.p.x(), c.p.y(), c.p.z())
        if self.successful:
            print "grips_db[-1].success()"

    def toStr(self):
        line = str(self.successful) + ' ' + str(len(self.contacts))
        for c in self.contacts:
            q = c.M.GetQuaternion()
            line += ' ' + str(c.p.x()) + ' ' + str(c.p.y()) + ' ' + str(c.p.z())
            line += ' ' + str(q[0]) + ' ' + str(q[1]) + ' ' + str(q[2]) + ' ' + str(q[3])
        return line

    def fromStr(self, line):
        tab = line.split()
        self.successful = bool(tab[0])
        contacts_count = int(tab[1])
        self.contacts = []
        for idx in range(0, contacts_count):
            px = float(tab[2 + idx*7 + 0])
            py = float(tab[2 + idx*7 + 1])
            pz = float(tab[2 + idx*7 + 2])
            qx = float(tab[2 + idx*7 + 3])
            qy = float(tab[2 + idx*7 + 4])
            qz = float(tab[2 + idx*7 + 5])
            qw = float(tab[2 + idx*7 + 6])
            self.contacts.append(PyKDL.Frame(PyKDL.Rotation.Quaternion(qx, qy, qz, qw), PyKDL.Vector(px,py,pz)))

def gripDist_new(a, b):
    fr_a = []
    T_O_Com = PyKDL.Frame(a.grasped_object.com)
    T_Com_O = T_O_Com.Inverse()
    for T_O_C in a.contacts:
        T_Com_C = T_Com_O * T_O_C
        fr_a.append( T_Com_C )

    fr_b = []
    for T_O_C in b.contacts:
        T_Com_C = T_Com_O * T_O_C
        fr_b.append( T_Com_C )

    if len(fr_a) >= len(fr_b):
        fr_0 = fr_a
        fr_1 = fr_b
    else:
        fr_0 = fr_b
        fr_1 = fr_a

    # fr_0 has more or equal number of contacts than fr_1
    for i in range(0, len(fr_0)):
        for j in range(0, len(fr_0)):
            if i == j:
                continue

def gripDist2(a, b):
    def estTransform(l1, l2):
        pos1 = []
        n1 = []
        def calculateScore(p1, p2, n1, n2):
            angle_score = 2.0*velmautils.getAngle(n1,n2)/math.pi
            pos_score = 10.0*(p1-p2).Norm()
            return pos_score, angle_score
        for f in l1:
            pos1.append( f * PyKDL.Vector() )
            n1.append( PyKDL.Frame(f.M) * PyKDL.Vector(0,0,1) )
        def calc_R(xa, ya, za):
            ret = []
            """ calculate the minimum distance of each contact point from jar surface pt """
            t = PyKDL.Frame(PyKDL.Rotation.RotX(xa)) * PyKDL.Frame(PyKDL.Rotation.RotY(ya)) * PyKDL.Frame(PyKDL.Rotation.RotX(za))
            index1 = 0
            for f in l2:
                dest_f = t * f
                pos2 = dest_f * PyKDL.Vector()
                n2 = PyKDL.Frame(dest_f.M) * PyKDL.Vector(0,0,1)
                s1, s2 = calculateScore(pos1[index1], pos2, n1[index1], n2)
                ret.append(s1)
                ret.append(s2)
                index1 += 1
            return np.array(ret)
        def f_2(c):
            """ calculate the algebraic distance between each contact point and jar surface pt """
            Di = calc_R(*c)
            return Di
        def sumf_2(p):
            return math.fsum(f_2(p)**2)
        angles_estimate = 0.0, 0.0, 0.0
        # least squares with constraints
        angles_2 = optimize.fmin_slsqp(sumf_2, angles_estimate, bounds=[(-math.pi, math.pi),(-math.pi, math.pi),(-math.pi, math.pi)], iprint=0)
        # least squares without constraints
#        angles_2, ier = optimize.leastsq(f_2, angles_estimate, maxfev = 1000)
        t = PyKDL.Frame(PyKDL.Rotation.RotX(angles_2[0])) * PyKDL.Frame(PyKDL.Rotation.RotY(angles_2[1])) * PyKDL.Frame(PyKDL.Rotation.RotX(angles_2[2]))
        index1 = 0
        score = 0.0
        ret = []
        for f in l2:
            dest_f = t * f
            pos2 = dest_f * PyKDL.Vector()
            n2 = PyKDL.Frame(dest_f.M) * PyKDL.Vector(0,0,1)
            s1, s2 = calculateScore(pos1[index1], pos2, n1[index1], n2)
            ret.append(s1)
            ret.append(s2)
            index1 += 1
        return math.fsum(np.array(ret)**2), angles_2

    fr_a = []
    T_O_Com = PyKDL.Frame(a.grasped_object.com)
    T_Com_O = T_O_Com.Inverse()
    for T_O_C in a.contacts:
        T_Com_C = T_Com_O * T_O_C
        fr_a.append( T_Com_C )

    fr_b = []
    for T_O_C in b.contacts:
        T_Com_C = T_Com_O * T_O_C
        fr_b.append( T_Com_C )

    if len(fr_a) >= len(fr_b):
        fr_0 = fr_a
        fr_1 = fr_b
    else:
        fr_0 = fr_b
        fr_1 = fr_a

    min_score = 10000.0
    min_angles = None
    # estimate for each permutation of the smaller set
    for it in itertools.permutations(fr_1):
        score, angles = estTransform(fr_0, it)
        if score < min_score:
            min_score = score
            min_angles = angles
    return min_score, min_angles, None, None, None, None, None, None, None

def gripDist3(a, b):
    def estTransform(l1, l2):
        pos1_list = []
        n1_list = []
        for f in l1:
            pos1_list.append( f * PyKDL.Vector() )
            n1_list.append( PyKDL.Frame(f.M) * PyKDL.Vector(0,0,1) )

        def calculateScore(p1_list, p2, n1_list, n2):
            min_score = None
            for i in range(0, len(p1_list)):
                p1 = p1_list[i]
                n1 = n1_list[i]
                angle_score = 2.0*velmautils.getAngle(n1,n2)/math.pi
                pos_score = 10.0*(p1-p2).Norm()
                score = angle_score * angle_score + pos_score * pos_score
                if min_score == None or min_score > score:
                    min_score = score
            return math.sqrt(min_score)
        def calc_R(xa, ya, za):
            ret = []
            t = PyKDL.Frame(PyKDL.Rotation.RotX(xa)) * PyKDL.Frame(PyKDL.Rotation.RotY(ya)) * PyKDL.Frame(PyKDL.Rotation.RotX(za))
            index1 = 0
            for f in l2:
                dest_f = t * f
                pos2 = dest_f * PyKDL.Vector()
                n2 = PyKDL.Frame(dest_f.M) * PyKDL.Vector(0,0,1)
                score = calculateScore(pos1_list, pos2, n1_list, n2)
                ret.append(score)
                index1 += 1
            return np.array(ret)
        def f_2(c):
            Di = calc_R(*c)
            return Di
        def sumf_2(p):
            return math.fsum(f_2(p)**2)


        angles_estimate = 0.0, 0.0, 0.0
        # least squares with constraints
        angles_2 = optimize.fmin_slsqp(sumf_2, angles_estimate, bounds=[(-math.pi, math.pi),(-math.pi, math.pi),(-math.pi, math.pi)], iprint=0)
        # least squares without constraints
#        angles_2, ier = optimize.leastsq(f_2, angles_estimate, maxfev = 1000)

        return sumf_2(angles_2), angles_2

    fr_a = []
    T_O_Com = PyKDL.Frame(a.grasped_object.com)
    T_Com_O = T_O_Com.Inverse()
    for T_O_C in a.contacts:
        T_Com_C = T_Com_O * T_O_C
        fr_a.append( T_Com_C )

    fr_b = []
    for T_O_C in b.contacts:
        T_Com_C = T_Com_O * T_O_C
        fr_b.append( T_Com_C )

    if len(fr_a) >= len(fr_b):
        fr_0 = fr_a
        fr_1 = fr_b
    else:
        fr_0 = fr_b
        fr_1 = fr_a

    score, angles = estTransform(fr_0, fr_1)

    return score, angles, None, None, None, None, None, None, None

def gripUnitTestVisual(sim_grips, openrave, object_name):
    for idx in range(0, len(sim_grips), 10):
        if sim_grips[idx] == None:
            continue
        print "idx: %s"%(idx)
        dists = []
        for idx2 in range(0, len(sim_grips)):
            if idx == idx2:
                continue
            if sim_grips[idx2] == None:
                continue
            dist, angles, all_scores, all_angles, all_scores2, n1_s_list, pos1_s_list, n2_s_list, pos2_s_list = gripDist3(sim_grips[idx], sim_grips[idx2])
            dists.append([dist, idx2])

        dists_sorted = sorted(dists, key=operator.itemgetter(0))

        grasp = openrave.getGrasp(object_name, idx)
        openrave.showGrasp(object_name, grasp)

        for i in range(0, 50):
            best_idx = dists_sorted[i][1]
            score = dists_sorted[i][0]
            print "    similar grasp #%s:  id: %s   score: %s"%(i, best_idx, score)
            grasp = openrave.getGrasp(object_name, best_idx)
            openrave.showGrasp(object_name, grasp)



def gripUnitTest(obj_grasp):
    print "gripUnitTest begin"
    # distance between identical grips
    print "distance between identical grips"
    grip1 = Grip(obj_grasp)
    grip1.addContact(PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi), PyKDL.Vector(-0.1,-0.03,0)))
    grip1.addContact(PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi), PyKDL.Vector(-0.05,-0.03,0)))
    grip1.addContact(PyKDL.Frame(PyKDL.Rotation.RotX(-90.0/180.0*math.pi), PyKDL.Vector(-0.075,0.03,0)))
    grip2 = copy.deepcopy(grip1)
    print gripDist(grip1, grip2)

    # distance between identical grips rotated in com
    print "distance between identical grips rotated in com"
    grip2 = copy.deepcopy(grip1)
    for i in range(0, len(grip2.contacts)):
        grip2.contacts[i] = PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi)) * grip2.contacts[i]
    print gripDist(grip1, grip2)

    # distance between identical grips rotated in com in 2 directions
    print "distance between identical grips rotated in com in 2 directions"
    grip2 = copy.deepcopy(grip1)
    for i in range(0, len(grip2.contacts)):
        grip2.contacts[i] = PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotZ(20.0/180.0*math.pi)) * grip2.contacts[i]
    print gripDist(grip1, grip2)

    # distance between identical grips rotated in com in 2 directions, one grip has additional contact
    print "distance between identical grips rotated in com in 2 directions, one grip has additional contact"
    grip2 = copy.deepcopy(grip1)
    for i in range(0, len(grip2.contacts)):
        grip2.contacts[i] = PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotZ(20.0/180.0*math.pi)) * grip2.contacts[i]
    grip2.addContact(PyKDL.Frame(PyKDL.Rotation.RotX(-90.0/180.0*math.pi), PyKDL.Vector(-0.01,0.03,0)))
    print gripDist(grip1, grip2)

    # distance between identical grips rotated in com in 2 directions, one grip has a bit diffrent one contact pos
    print "distance between identical grips rotated in com in 2 directions, one grip has a bit diffrent one contact pos"
    grip2 = copy.deepcopy(grip1)
    grip2.contacts[0] = PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi), PyKDL.Vector(-0.11,-0.03,0))
    print gripDist(grip1, grip2)

    # distance between identical grips rotated in com in 2 directions, one grip has a bit diffrent one contact rot
    print "distance between identical grips rotated in com in 2 directions, one grip has a bit diffrent one contact rot"
    grip2 = copy.deepcopy(grip1)
    grip2.contacts[0] = PyKDL.Frame(PyKDL.Rotation.RotX(70.0/180.0*math.pi), PyKDL.Vector(-0.1,-0.03,0))
    print gripDist(grip1, grip2)

    # distance between identical grips rotated in conter of the object, with different com
    print "distance between identical grips rotated in conter of the object, with different com"
    com = copy.deepcopy(obj_grasp.com)
    obj_grasp.com = PyKDL.Vector(0,-0.01,0)
    grip1.grasped_object = obj_grasp
    grip2 = copy.deepcopy(grip1)
    for i in range(0, len(grip2.contacts)):
        grip2.contacts[i] = PyKDL.Frame(PyKDL.Rotation.RotX(-90.0/180.0*math.pi)) * grip2.contacts[i]
    print gripDist(grip1, grip2)
    obj_grasp.com = com

    print "gripUnitTest end"

