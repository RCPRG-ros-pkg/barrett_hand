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
import rospkg

import PyKDL
import math
from numpy import *
import numpy as np
import copy
import matplotlib.pyplot as plt
import thread
from velma import Velma
import random
from openravepy import *
#from ..openravepy_int import KinBody, TriMesh
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

import subprocess
import ode
import xode.transform

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

                    self.openrave_robot.GetActiveManipulator().SetChuckingDirection([-1,0,1,1])#directions)
                    print "manipulator name: " + self.openrave_robot.GetActiveManipulator().GetName()
                    target = self.env.GetKinBody("object")
                    self.openrave_robot.SetDOFValues(pos)
                    raw_input(".")

                    contacts,finalconfig,mindist,volume = self.grasper.Grasp(execute=False, outputfinal=True, transformrobot=False, target=target)

                    print finalconfig
                    self.openrave_robot.SetDOFValues(finalconfig[0])
                    raw_input(",")
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

#        for contact_idx in range(len(contacts)):
#            print contact_planes[contact_idx]

#        for pl_idx in range(len(qhullplanes)):
#            print qhullplanes_contacts[pl_idx]

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



    def createInteractiveMarkerControl6DOF(self, mode, axis):
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED
        if mode == InteractiveMarkerControl.ROTATE_AXIS:
            control.name = 'rotate_';
        if mode == InteractiveMarkerControl.MOVE_AXIS:
            control.name = 'move_';
        if axis == 'x':
            control.orientation = Quaternion(1,0,0,1)
            control.name = control.name+'x';
        if axis == 'y':
            control.orientation = Quaternion(0,1,0,1)
            control.name = control.name+'x';
        if axis == 'z':
            control.orientation = Quaternion(0,0,1,1)
            control.name = control.name+'x';
        control.interaction_mode = mode
        return control

    def processFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and feedback.control_name == "button":
            self.run_ode_simulation = True

        if feedback.marker_name == 'obj_pose_marker':# and feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.T_W_O = pm.fromMsg(feedback.pose)

    def createButtonMarkerControl(self, scale, position):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale = scale
        marker.pose.position = position
        marker.color = ColorRGBA(1,0,0,1)
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def insert6DofGlobalMarker(self, T_W_M):
        int_position_marker = InteractiveMarker()
        int_position_marker.header.frame_id = 'world'
        int_position_marker.name = 'obj_pose_marker'
        int_position_marker.scale = 0.1
        int_position_marker.pose = pm.toMsg(T_W_M)
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.ROTATE_AXIS,'z'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'x'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'y'));
        int_position_marker.controls.append(self.createInteractiveMarkerControl6DOF(InteractiveMarkerControl.MOVE_AXIS,'z'));
        self.mk_server.insert(int_position_marker, self.processFeedback)

        int_button_marker = InteractiveMarker()
        int_button_marker.header.frame_id = 'world'
        int_button_marker.name = 'obj_button_marker'
        int_button_marker.scale = 0.2
        int_button_marker.pose = pm.toMsg(PyKDL.Frame())
        box = self.createButtonMarkerControl(Point(0.05,0.05,0.05), Point(0.0, 0.0, 0.15) )
        box.interaction_mode = InteractiveMarkerControl.BUTTON
        box.name = 'button'
        int_button_marker.controls.append( box )
        self.mk_server.insert(int_button_marker, self.processFeedback)

        self.mk_server.applyChanges()

    def erase6DofMarker(self):
        self.mk_server.erase('obj_pose_marker')
        self.mk_server.applyChanges()

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

    def getQualityMeasure(self, qhull):
        if qhull == None:
            return 0.0

        mindist = None
        for qp in qhull:
            if qp[6] >= 0:
                return 0
            if mindist == None or mindist > -qp[6]:
                mindist = -qp[6]

        return mindist

    def getQualityMeasure2(self, qhull, wr):
        if qhull == None:
            return 0.0

        wr6 = [wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]]
        mindist = None
        for qp in qhull:
            n = np.array([qp[0],qp[1],qp[2],qp[3],qp[4],qp[5]])
            if np.dot(n,n) > 1.00001 or np.dot(n,n) < 0.9999:
                print "ERROR: getQualityMeasure2: np.dot(n,n): %s"%(np.dot(n,n))
                exit(0)
            dot = np.dot(np.array(wr6), n)
            if dot > 0:
                dqp = -qp[6]/dot
                if mindist == None or mindist > dqp:
                    mindist = dqp
        return mindist

    def transformKdlToOde(self, Tkdl):
        return None
#        print Tkdl
        Tode = xode.transform.Transform()
        for i in range(3):
            for j in range(3):
                Tode.m[j][i] = Tkdl[(j,i)]
        Tode.m[3][0] = Tkdl[(0,3)]
        Tode.m[3][1] = Tkdl[(1,3)]
        Tode.m[3][2] = Tkdl[(2,3)]
#        print Tode.m
        return Tode

    def transformOdeToKdl(self, Tode):
        return None
#        rot = PyKDL.Rotation(
#        Tode.m[0][0], Tode.m[0][1], Tode.m[0][2],
#        Tode.m[1][0], Tode.m[1][1], Tode.m[1][2], 
#        Tode.m[2][0], Tode.m[2][1], Tode.m[2][2])
        rot = PyKDL.Rotation(
        Tode.m[0][0], Tode.m[1][0], Tode.m[2][0],
        Tode.m[0][1], Tode.m[1][1], Tode.m[2][1], 
        Tode.m[0][2], Tode.m[1][2], Tode.m[2][2])
        pos = PyKDL.Vector(Tode.m[3][0], Tode.m[3][1], Tode.m[3][2])
        Tkdl = PyKDL.Frame(rot, pos)
#        print Tkdl
        return Tkdl

    def setOdeBodyPose(self, body, T):
        # in kdl: (x,y,z,w)
        # in ode: (w,x,y,z)
        q = T.M.GetQuaternion()
        body.setPosition( (T.p.x(), T.p.y(), T.p.z()) )
        body.setQuaternion( (q[3], q[0], q[1], q[2]) )
#        body.setRotation((
#        T.M[(0,0)],T.M[(0,1)],T.M[(0,2)],
#        T.M[(1,0)],T.M[(1,1)],T.M[(1,2)],
#        T.M[(2,0)],T.M[(2,1)],T.M[(2,2)] ))
#        body.setRotation((
#        T.M[(0,0)],T.M[(1,0)],T.M[(2,0)],
#        T.M[(0,1)],T.M[(1,1)],T.M[(2,1)],
#        T.M[(0,2)],T.M[(1,2)],T.M[(2,2)] ))

    def getOdeBodyPose(self, body):
        # in kdl: (x,y,z,w)
        # in ode: (w,x,y,z)
        pos = body.getPosition()
        rot = body.getRotation()
#        q = body.getQuaternion()
#        return PyKDL.Frame(PyKDL.Rotation.Quaternion(q[1], q[2], q[3], q[0]), PyKDL.Vector(pos[0], pos[1], pos[2]))
        return PyKDL.Frame(PyKDL.Rotation(
        rot[0], rot[1], rot[2],
        rot[3], rot[4], rot[5],
        rot[6], rot[7], rot[8]),
        PyKDL.Vector(pos[0], pos[1], pos[2]))

    # Collision callback
    def near_callback(self, args, geom1, geom2):
        """Callback function for the collide() method.

        This function checks if the given geoms do collide and
        creates contact joints if they do.
        """

        body1 = geom1.getBody()
        body2 = geom2.getBody()

        if ode.areConnected(body1, body2):
            return

        # Check if the objects do collide
        contacts = ode.collide(geom1, geom2)

        if geom1.name == "object":
            for c in contacts:
                pos, normal, depth, g1, g2 = c.getContactGeomParams()
                self.grasp_contacts.append( (pos[0], pos[1], pos[2], -normal[0], -normal[1], -normal[2]) )

        if geom2.name == "object":
            for c in contacts:
                pos, normal, depth, g1, g2 = c.getContactGeomParams()
                self.grasp_contacts.append( (pos[0], pos[1], pos[2], normal[0], normal[1], normal[2]) )

        # Create contact joints
        world,contactgroup = args
        for c in contacts:
            c.setBounce(0.0)
            c.setMu(5000)
            j = ode.ContactJoint(world, contactgroup, c)
            j.attach(body1, body2)

    def spin(self):
        if True:
            # Create a world object
            world = ode.World()
#            world.setGravity( (0,0,-3.81) )
            world.setGravity( (0,0,0) )
            CFM = world.getCFM()
            ERP = world.getERP()
            print "CFM: %s  ERP: %s"%(CFM, ERP)
#            world.setCFM(0.001)
#            print "CFM: %s  ERP: %s"%(CFM, ERP)



        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

        #
        # Init Openrave
        #
        parser = OptionParser(description='Openrave Velma interface')
        OpenRAVEGlobalArguments.addOptions(parser)
        (options, leftargs) = parser.parse_args()
        self.env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)

#        self.openrave_robot = self.env.ReadRobotXMLFile('robots/barretthand_ros.robot.xml')

        mimic_joints = [
        ("right_HandFingerTwoKnuckleOneJoint", "right_HandFingerOneKnuckleOneJoint*1.0", "|right_HandFingerOneKnuckleOneJoint 1.0", ""),
        ("right_HandFingerOneKnuckleThreeJoint", "right_HandFingerOneKnuckleTwoJoint*0.33333", "|right_HandFingerOneKnuckleTwoJoint 0.33333", ""),
        ("right_HandFingerTwoKnuckleThreeJoint", "right_HandFingerTwoKnuckleTwoJoint*0.33333", "|right_HandFingerTwoKnuckleTwoJoint 0.33333", ""),
        ("right_HandFingerThreeKnuckleThreeJoint", "right_HandFingerThreeKnuckleTwoJoint*0.33333", "|right_HandFingerThreeKnuckleTwoJoint 0.33333", ""),
        ]

        rospack = rospkg.RosPack()

        self.urdf_module = RaveCreateModule(self.env, 'urdf')

        xacro_uri = rospack.get_path('barrett_hand_defs') + '/robots/barrett_hand.urdf.xml'

        urdf_uri = '/tmp/barrett_hand.urdf'
        srdf_uri = rospack.get_path('barrett_hand_defs') + '/robots/barrett_hand.srdf'
        arg1 = "collision_model_full:=true"
        arg2 = "collision_model_simplified:=false"
        arg3 = "collision_model_enlargement:=0.0"
        arg4 = "collision_model_no_hands:=false"

        subprocess.call(["rosrun", "xacro", "xacro", "-o", urdf_uri, xacro_uri, arg1, arg2, arg3, arg4])
        robot_name = self.urdf_module.SendCommand('load ' + urdf_uri + ' ' + srdf_uri )
        print "robot name: " + robot_name
        self.openrave_robot = self.env.GetRobot(robot_name)

        self.env.Remove(self.openrave_robot)

        for mimic in mimic_joints:
            mj = self.openrave_robot.GetJoint(mimic[0])
            mj.SetMimicEquations(0, mimic[1], mimic[2], mimic[3])
        self.openrave_robot.GetActiveManipulator().SetChuckingDirection([0,0,0,0])#directions)
        self.env.Add(self.openrave_robot)
        self.openrave_robot = self.env.GetRobot(robot_name)

#        print "robots: "
#        print self.env.GetRobots()

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

        vertices, faces = surfaceutils.readStl(rospack.get_path('velma_scripts') + "/data/meshes/klucz_gerda_ascii.stl", scale=1.0)
#        vertices, faces = surfaceutils.readStl("klucz_gerda_ascii.stl", scale=1.0)
        self.addTrimesh("object", vertices, faces)

#        self.addBox("object", 0.2,0.06,0.06)
#        self.addSphere("object", 0.15)
#        vertices, faces = self.getMesh("object")

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

        #
        # definition of the grasps
        #
        grasp_id = 0
        if grasp_id == 0:
            grasp_direction = [0, 1, -1, 1]    # spread, f1, f3, f2
            grasp_initial_configuration = [60.0/180.0*math.pi, None, None, None]
            self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.122103662206, -0.124395758838, -0.702726011729, 0.689777190329), PyKDL.Vector(-0.00115787237883, -0.0194999426603, 0.458197712898))
#            self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.174202588426, -0.177472708083, -0.691231954612, 0.678495061771), PyKDL.Vector(0.0, -0.0213436260819, 0.459123969078))
        elif grasp_id == 1:
            grasp_direction = [0, 1, -1, 1]    # spread, f1, f3, f2
            grasp_initial_configuration = [90.0/180.0*math.pi, None, None, None]
            self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.0187387771868, -0.708157209758, -0.0317875569224, 0.705090018033), PyKDL.Vector(4.65661287308e-10, 0.00145332887769, 0.472836345434))
        elif grasp_id == 2:
            grasp_direction = [0, 1, 0, 0]    # spread, f1, f3, f2
            grasp_initial_configuration = [90.0/180.0*math.pi, None, 90.0/180.0*math.pi, 0]
            self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.0187387763947, -0.708157179826, -0.0317875555789, 0.705089928626), PyKDL.Vector(0.0143095180392, 0.00145332887769, 0.483659058809))
        elif grasp_id == 3:
            grasp_direction = [0, 1, 1, 1]    # spread, f1, f3, f2
            grasp_initial_configuration = [90.0/180.0*math.pi, None, None, None]
            self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.00518634245761, -0.706548316769, -0.0182458505507, 0.707410947861), PyKDL.Vector(0.000126354629174, -0.00217361748219, 0.47637796402))
        elif grasp_id == 4:
            grasp_direction = [0, 0, 1, 0]    # spread, f1, f3, f2
            grasp_initial_configuration = [90.0/180.0*math.pi, 100.0/180.0*math.pi, None, 100.0/180.0*math.pi]
            self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.153445252933, -0.161230275653, 0.681741576082, 0.696913201022), PyKDL.Vector(0.000126355327666, 0.00152841210365, 0.466048002243))
        elif grasp_id == 5:
            grasp_direction = [0, 0, 1, 0]    # spread, f1, f3, f2
            grasp_initial_configuration = [100.0/180.0*math.pi, 101.5/180.0*math.pi, None, 101.5/180.0*math.pi]
            self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.155488650062, -0.159260521271, 0.690572597636, 0.688163302213), PyKDL.Vector(-0.000278688268736, 0.00575117766857, 0.461560428143))
        elif grasp_id == 6:
            grasp_direction = [0, 1, -1, 1]    # spread, f1, f3, f2
            grasp_initial_configuration = [90.0/180.0*math.pi, None, 0, 0]
            self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.512641041738, -0.485843507183, -0.514213889193, 0.48655882699), PyKDL.Vector(-0.000278423947748, -0.00292747467756, 0.445628076792))
        else:
            print "ERROR: unknown grasp_id: %s"%(grasp_id)
            exit(0)

        self.updatePose("object", self.T_W_O)

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
            def getDirectionIndex(n):
                min_angle = -45.0/180.0*math.pi
                angle_range = 90.0/180.0*math.pi
                angles_count = 5
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
                    print sec, a1i, a2i
                    a1i = 0
                if a1i >= angles_count:
#                    print sec, a1i, a2i
                    a1i = angles_count-1
                if a2i < 0:
                    print sec, a1i, a2i
                    a2i = 0
                if a2i >= angles_count:
                    print sec, a1i, a2i
                    a2i = angles_count-1
                return sec * angles_count2 + a1i * angles_count + a2i

            # generate a dictionary of indexed directions
            normals_sphere_indexed_dir = velmautils.generateNormalsSphere(3.0/180.0*math.pi)
            print len(normals_sphere_indexed_dir)
            indices_directions = {}
            for n in normals_sphere_indexed_dir:
                index = getDirectionIndex(n)
                if not index in indices_directions:
                    indices_directions[index] = [n]
                else:
                    indices_directions[index].append(n)

            for index in indices_directions:
                n_mean = PyKDL.Vector()
                for n in indices_directions[index]:
                    n_mean += n
                n_mean.Normalize()
                indices_directions[index] = n_mean

            # test direction indexing
            if False:
                normals_sphere = velmautils.generateNormalsSphere(3.0/180.0*math.pi)
                print len(normals_sphere)
                m_id = 0
                for current_index in range(5*5*6):
                  count = 0
                  r = random.random()
                  g = random.random()
                  b = random.random()
                  for n in normals_sphere:
                    sec = -1
                    a1 = None
                    a2 = None
                    index = getDirectionIndex(n)
                    if index == current_index:
                        m_id = self.pub_marker.publishSinglePointMarker(n*0.1, m_id, r=r, g=g, b=b, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.007, 0.007, 0.007), T=None)
                        count += 1
                  print current_index, count
                for index in indices_directions:
                    m_id = self.pub_marker.publishSinglePointMarker(indices_directions[index]*0.12, m_id, r=1, g=1, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.012, 0.012, 0.012), T=None)

                exit(0)

        #
        # PyODE test
        #
        if True:

            fixed_joints_names_for_fixed_DOF = [
            ["right_HandFingerOneKnuckleOneJoint", "right_HandFingerTwoKnuckleOneJoint"],          # spread
            ["right_HandFingerOneKnuckleTwoJoint", "right_HandFingerOneKnuckleThreeJoint"],        # f1
            ["right_HandFingerThreeKnuckleTwoJoint", "right_HandFingerThreeKnuckleThreeJoint"],    # f3
            ["right_HandFingerTwoKnuckleTwoJoint", "right_HandFingerTwoKnuckleThreeJoint"],        # f2
            ]
            coupled_joint_names_for_fingers = ["right_HandFingerOneKnuckleThreeJoint", "right_HandFingerTwoKnuckleThreeJoint", "right_HandFingerThreeKnuckleThreeJoint"]

            actuated_joints_for_DOF = [
            ["right_HandFingerOneKnuckleOneJoint", "right_HandFingerTwoKnuckleOneJoint"],  # spread
            ["right_HandFingerOneKnuckleTwoJoint"],                                        # f1
            ["right_HandFingerThreeKnuckleTwoJoint"],                                      # f3
            ["right_HandFingerTwoKnuckleTwoJoint"]]                                        # f2

            ignored_links = ["world", "world_link"]

            self.run_ode_simulation = False
            self.insert6DofGlobalMarker(self.T_W_O)

            print "grasp_direction = [%s, %s, %s, %s]    # spread, f1, f3, f2"%(grasp_direction[0], grasp_direction[1], grasp_direction[2], grasp_direction[3])
            print "grasp_initial_configuration = [%s, %s, %s, %s]"%(grasp_initial_configuration[0], grasp_initial_configuration[1], grasp_initial_configuration[2], grasp_initial_configuration[3])
            print "grasp_final_configuration = %s"%(self.openrave_robot.GetDOFValues())
            rot_q = self.T_W_O.M.GetQuaternion()
            print "self.T_W_O = PyKDL.Frame(PyKDL.Rotation.Quaternion(%s, %s, %s, %s), PyKDL.Vector(%s, %s, %s))"%(rot_q[0], rot_q[1], rot_q[2], rot_q[3], self.T_W_O.p.x(), self.T_W_O.p.y(), self.T_W_O.p.z())

            self.erase6DofMarker()
            raw_input("Press ENTER to continue...")

            # reset the gripper in Openrave
            self.openrave_robot.SetDOFValues([0,0,0,0])

            # read the position of all links
            grasp_links_poses = {}
            for link in self.openrave_robot.GetLinks():
                grasp_links_poses[link.GetName()] = self.OpenraveToKDL(link.GetTransform())

#            print "obtained the gripper configuration for the grasp:"
#            print finalconfig[0]

            #
            # simulation in ODE
            #

            # Create a world object
            world = ode.World()
#            world.setGravity( (0,0,-3.81) )
            world.setGravity( (0,0,0) )
            CFM = world.getCFM()
            ERP = world.getERP()
            print "CFM: %s  ERP: %s"%(CFM, ERP)
#            world.setCFM(0.001)
#            print "CFM: %s  ERP: %s"%(CFM, ERP)

            self.space = ode.Space()

            # Create a body inside the world
            body = ode.Body(world)
            M = ode.Mass()
            M.setCylinderTotal(0.02, 1, 0.005, 0.09)
            body.setMass(M)

            ode_mesh = ode.TriMeshData()
            ode_mesh.build(vertices, faces)
            geom = ode.GeomTriMesh(ode_mesh, self.space)
            geom.name = "object"
            geom.setBody(body)

            self.setOdeBodyPose(geom, self.T_W_O)

            ode_gripper_geoms = {}
            for link in self.openrave_robot.GetLinks():

                link_name = link.GetName()
                if link_name in ignored_links:
                    print "ignoring: %s"%(link_name)
                    continue
                print "adding: %s"%(link_name)

                ode_mesh_link = None
                body_link = None

                T_W_L = self.OpenraveToKDL(link.GetTransform())

                col = link.GetCollisionData()
                vertices = col.vertices
                faces = col.indices
                ode_mesh_link = ode.TriMeshData()
                ode_mesh_link.build(vertices, faces)
                ode_gripper_geoms[link_name] = ode.GeomTriMesh(ode_mesh_link, self.space)

                if True:
                    body_link = ode.Body(world)
                    M_link = ode.Mass()
                    M_link.setCylinderTotal(0.05, 1, 0.01, 0.09)
                    body_link.setMass(M_link)
                    ode_gripper_geoms[link_name].setBody(body_link)
                    ode_gripper_geoms[link_name].name = link.GetName()
                    self.setOdeBodyPose(body_link, T_W_L)

            actuated_joint_names = []
            for dof in range(4):
                for joint_name in actuated_joints_for_DOF[dof]:
                    actuated_joint_names.append(joint_name)

            ode_gripper_joints = {}
            for joint_name in joint_names:
                joint = self.openrave_robot.GetJoint(joint_name)
                link_parent = joint.GetHierarchyParentLink().GetName()
                link_child = joint.GetHierarchyChildLink().GetName()
                if link_parent in ode_gripper_geoms and link_child in ode_gripper_geoms:
                    axis = joint.GetAxis()
                    anchor = joint.GetAnchor()
                    if joint_name in actuated_joint_names:
                        ode_gripper_joints[joint_name] = ode.AMotor(world)
                        ode_gripper_joints[joint_name].attach(ode_gripper_geoms[link_parent].getBody(), ode_gripper_geoms[link_child].getBody())
                        ode_gripper_joints[joint_name].setMode(ode.AMotorUser)#Euler)
#                        ode_gripper_joints[joint_name].setNumAxes(3)
                        ode_gripper_joints[joint_name].setAxis(0,1,axis)
#                        ode_gripper_joints[joint_name].setAxis(1,1,axis)
#                        ode_gripper_joints[joint_name].setAxis(2,1,axis)
                        ode_gripper_joints[joint_name].setParam(ode.ParamFMax, 10.0)
#                        ode_gripper_joints[joint_name].setParam(ode.ParamFMax2, 10.0)
#                        ode_gripper_joints[joint_name].setParam(ode.ParamFMax3, 10.0)
                    else:
                        ode_gripper_joints[joint_name] = ode.HingeJoint(world)
                        ode_gripper_joints[joint_name].attach(ode_gripper_geoms[link_parent].getBody(), ode_gripper_geoms[link_child].getBody())
                        ode_gripper_joints[joint_name].setAxis(-axis)
                        ode_gripper_joints[joint_name].setAnchor(anchor)

#                    ode_gripper_joints[joint_name] = ode.HingeJoint(world)
#                    ode_gripper_joints[joint_name].attach(ode_gripper_geoms[link_parent].getBody(), ode_gripper_geoms[link_child].getBody())
#                    ode_gripper_joints[joint_name].setAxis(-axis)
#                    ode_gripper_joints[joint_name].setAnchor(anchor)

#                    if joint_name in actuated_joint_names:
#                        ode_gripper_joints[joint_name].setParam(ode.ParamFMax, 10.0)

                    limits = joint.GetLimits()
                    value = joint.GetValue(0)
#                    print joint_name
#                    print "limits: %s %s"%(limits[0], limits[1])
#                    print "axis: %s"%(axis)
#                    print "anchor: %s"%(anchor)
#                    print "value: %s"%(value)


                    lim = [limits[0], limits[1]]
                    if limits[0] <= -math.pi:
#                        print "lower joint limit %s <= -PI, setting to -PI+0.01"%(limits[0])
                        lim[0] = -math.pi + 0.01
                    if limits[1] >= math.pi:
#                        print "upper joint limit %s >= PI, setting to PI-0.01"%(limits[1])
                        lim[1] = math.pi - 0.01
                    ode_gripper_joints[joint_name].setParam(ode.ParamLoStop, lim[0])
                    ode_gripper_joints[joint_name].setParam(ode.ParamHiStop, lim[1])
#                    ode_gripper_joints[joint_name].setParam(ode.ParamFudgeFactor, 0.5)

            ode_fixed_joint = ode.FixedJoint(world)
            ode_fixed_joint.attach(None, ode_gripper_geoms["right_HandPalmLink"].getBody())
            ode_fixed_joint.setFixed()

            #
            # set the poses of all links as for the grasp
            #
            for link_name in grasp_links_poses:
                if link_name in ignored_links:
                    continue
                ode_body = ode_gripper_geoms[link_name].getBody()
                T_W_L = grasp_links_poses[link_name]
                self.setOdeBodyPose(ode_body, T_W_L)

            fixed_joint_names = []
            fixed_joint_names += coupled_joint_names_for_fingers
            for dof in range(4):
                if grasp_direction[dof] == 0.0:
                    for joint_name in fixed_joints_names_for_fixed_DOF[dof]:
                        if not joint_name in fixed_joint_names:
                            fixed_joint_names.append(joint_name)

            #
            # change all coupled joints to fixed joints
            #
            fixed_joints = {}
            for joint_name in fixed_joint_names:
                # save the bodies attached
                body0 = ode_gripper_joints[joint_name].getBody(0)
                body1 = ode_gripper_joints[joint_name].getBody(1)
                # save the joint angle
                if joint_name in actuated_joint_names:
                    angle = ode_gripper_joints[joint_name].getAngle(0)
                else:
                    angle = ode_gripper_joints[joint_name].getAngle()
                # detach the joint
                ode_gripper_joints[joint_name].attach(None, None)
                del ode_gripper_joints[joint_name]
                fixed_joints[joint_name] = [ode.FixedJoint(world), angle]
                fixed_joints[joint_name][0].attach(body0, body1)
                fixed_joints[joint_name][0].setFixed()

            # change all actuated joints to motor joints
#            actuated_joint_names = []
#            for dof in range(4):
#                for joint_name in actuated_joints_for_DOF[dof]:
#                    actuated_joint_names.append(joint_name)
#            for joint_name in actuated_joint_names:


            # A joint group for the contact joints that are generated whenever
            # two bodies collide
            contactgroup = ode.JointGroup()

            print "ode_gripper_geoms:"
            print ode_gripper_geoms

            initial_T_W_O = self.T_W_O
            # Do the simulation...
            dt = 0.001
            total_time = 0.0
            failure = False

            #
            # PID
            #
            Kc = 1.0
#            Ti = 1000000000000.0
            KcTi = 0.0
            Td = 0.0
            Ts = 0.001
            pos_d = 1.0
            e0 = 0.0
            e1 = 0.0
            e2 = 0.0
            u = 0.0
            u_max = 1.0
            while total_time < 5.0 and not rospy.is_shutdown():
                #
                # ODE simulation
                #

                joint = ode_gripper_joints["right_HandFingerOneKnuckleTwoJoint"]
                e2 = e1
                e1 = e0
                e0 = pos_d - joint.getAngle(0)
                u = u + Kc * (e0 - e1) + KcTi * Ts * e0 + Kc * Td / Ts * (e0 - 2.0 * e1 + e2)
                if u > u_max:
                    u = u_max
                if u < -u_max:
                    u = -u_max
                joint.setParam(ode.ParamFMax, 10000.0)
                joint.setParam(ode.ParamVel, 1.1)
#                joint.setParam(ode.ParamVel2, 1.1)
#                joint.setParam(ode.ParamVel3, 1.1)
                print joint.getAngle(0), "   ", u #, "   ", joint.getAngleRate(0)
#                print u

#                joint.addTorque(u)
#                for dof in range(4):
#                    for joint_name in actuated_joints_for_DOF[dof]:
#                        if joint_name in ode_gripper_joints:
#                            ode_gripper_joints[joint_name].addTorque(1*grasp_direction[dof])

                self.grasp_contacts = []
                self.space.collide((world,contactgroup), self.near_callback)
                world.step(dt)
                total_time += dt
                contactgroup.empty()

                #
                # ROS interface
                #

                old_m_id = m_id
                m_id = 0
                # publish frames from ODE
                if False:
                    for link_name in ode_gripper_geoms:
                        link_body = ode_gripper_geoms[link_name].getBody()
                        if link_body == None:
                            link_body = ode_gripper_geoms[link_name]
                        T_W_Lsim = self.getOdeBodyPose(link_body)
                        m_id = self.pub_marker.publishFrameMarker(T_W_Lsim, m_id, scale=0.05, frame='world', namespace='default')

                # publish the mesh of the object
                T_W_Osim = self.getOdeBodyPose(body)
                m_id = self.pub_marker.publishConstantMeshMarker("package://velma_scripts/data/meshes/klucz_gerda_binary.stl", m_id, r=1, g=0, b=0, scale=1.0, frame_id='world', namespace='default', T=T_W_Osim)

                # update the gripper visualization in ros
                js = JointState()
                js.header.stamp = rospy.Time.now()
                for jn in joint_map:
                    ros_joint_name = joint_map[jn]
                    js.name.append(ros_joint_name)
                    if jn in ode_gripper_joints:
                        if jn in actuated_joint_names:
                            js.position.append( ode_gripper_joints[jn].getAngle(0) )
                        else:
                            js.position.append( ode_gripper_joints[jn].getAngle() )
                    elif jn in fixed_joints:
                        js.position.append(fixed_joints[jn][1])
                    else:
                        js.position.append(0)
                self.pub_js.publish(js)

                # draw contacts
                for c in self.grasp_contacts:
                    cc = PyKDL.Vector(c[0], c[1], c[2])
                    cn = PyKDL.Vector(c[3], c[4], c[5])
                    m_id = self.pub_marker.publishVectorMarker(cc, cc+cn*0.04, m_id, 1, 0, 0, frame='world', namespace='default', scale=0.003)

                if m_id < old_m_id:
                    self.pub_marker.eraseMarkers(m_id,old_m_id+1, frame_id='world')

                diff_T_W_O = PyKDL.diff(initial_T_W_O, T_W_Osim)
#                if diff_T_W_O.vel.Norm() > 0.02 or diff_T_W_O.rot.Norm() > 30.0/180.0*math.pi:
#                    print "the object moved"
#                    print diff_T_W_O
#                    failure = True
#                    break
                rospy.sleep(0.01)

            if not failure:
                T_O_Wsim = T_W_Osim.Inverse()
                TR_O_Wsim = PyKDL.Frame(T_O_Wsim.M)
                contacts = []
                for c in self.grasp_contacts:
                    cc_W = PyKDL.Vector(c[0], c[1], c[2])
                    cn_W = PyKDL.Vector(c[3], c[4], c[5])
                    cc_O = T_O_Wsim * cc_W
                    cn_O = TR_O_Wsim * cn_W
                    contacts.append([cc_O[0], cc_O[1], cc_O[2], cn_O[0], cn_O[1], cn_O[2]])
                gws, contact_planes = self.generateGWS(contacts, 1.0)

                grasp_quality = None
                for wr in ext_wrenches:
                    wr_qual = self.getQualityMeasure2(gws, wr)
                    if grasp_quality == None or wr_qual < grasp_quality:
                        grasp_quality = wr_qual

                grasp_quality_classic = self.getQualityMeasure(gws)

                print "grasp_quality_classic: %s     grasp_quality: %s"%(grasp_quality_classic, grasp_quality)

            exit(0)


if __name__ == '__main__':

    rospy.init_node('grasp_leanring')

    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)

    task.spin()


