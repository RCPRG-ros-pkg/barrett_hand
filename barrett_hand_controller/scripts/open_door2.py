#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Robot Control and Pattern Recognition Group, Warsaw University of Technology
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of the <organization> nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYright HOLDERS AND CONTRIBUTORS "AS IS" AND
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
from geometry_msgs.msg import *
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *
from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *
import scipy.io as sio

import PyKDL
import math
from numpy import *
from scipy import optimize

class DoorOpener:
    """
Class for opening door with velma robot.
"""
    def tupleToPose(self, t):
        return Pose(Point(t[0][0], t[0][1], t[0][2]), Quaternion(t[1][0], t[1][1], t[1][2], t[1][3]))

    def PoseToTuple(self, p):
        return [p.position.x, p.position.y, p.position.z], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

    def get_pressure_sensors_info_client(self):
        service_name = '/' + self.prefix + '_hand/get_pressure_info'
        rospy.wait_for_service(service_name)
        try:
            get_pressure_sensors_info = rospy.ServiceProxy(service_name, BHGetPressureInfo)
            resp = get_pressure_sensors_info()
            return resp.info
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def alvarMarkerCallback(self, data):
        marker_count = len(data.markers)

        for i in range(0, marker_count):
            if data.markers[i].id == self.door_makrer_id:
                self.door_marker_pose = self.PoseToTuple(data.markers[i].pose.pose)
                self.door_marker_visible = True

    def tactileCallback(self, data):
        self.max_tactile_value = 0.0
        fingers = [data.finger1_tip, data.finger2_tip, data.finger3_tip]
        for f in range(0,3):
            for i in range(0, 24):
                if fingers[f][i] > self.max_tactile_value:
                    self.max_tactile_value = fingers[f][i]
                    self.max_tactile_index = i
                    self.max_tactile_finger = f

    def sendNextEvent(self):
        pc = PointStamped()
        pc.header.frame_id = 'torso_base'
        pc.header.stamp = rospy.Time.now()
        pc.point = Point(self.pub_msg_id, 0, 0)
        self.pub_msg.publish(pc)
        self.pub_msg_id += 1

    def __init__(self):
        self.prefix="right"
        # publisher with imu data
        self.pub_trajectory = rospy.Publisher("/"+self.prefix+"_arm/trajectory", CartesianTrajectory)
        self.pub_impedance = rospy.Publisher("/"+self.prefix+"_arm/impedance", CartesianImpedanceTrajectory)
        self.pub_circle = rospy.Publisher("/estimated_circle", QuaternionStamped)
        self.pub_pm = rospy.Publisher("/pm", PointStamped)
        self.pub_pc = rospy.Publisher("/pc", PointStamped)
        self.pub_msg = rospy.Publisher("/message", PointStamped)
        self.pub_msg_id = 0
        self.listener = tf.TransformListener();
        self.br = tf.TransformBroadcaster()

        self.pub_marker = rospy.Publisher('/door_markers', MarkerArray)

        rospy.sleep(1.0)
        
        self.door_makrer_id=3

        palm_msg = self.listener.lookupTransform(self.prefix+'_HandPalmLink', self.prefix+'_arm_7_link', rospy.Time(0))
        self.palm = pm.fromTf(palm_msg)
        self.door_marker_visible = False
        self.door_marker_pose = Pose()

        self.max_tactile_value = 0
        self.max_tactile_index = 0
        self.max_tactile_finger = 0

        print "Requesting pressure sensors info"
        self.pressure_info = self.get_pressure_sensors_info_client()

        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvarMarkerCallback)
        rospy.Subscriber('/'+self.prefix+'_hand/BHPressureState', BHPressureState, self.tactileCallback)

    def moveArm(self, gripper_pose):
        gripper = gripper_pose
        real_gripper = self.listener.lookupTransform('torso_base', self.prefix+'_HandPalmLink', rospy.Time(0))
        real_tool = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))
        p = pm.toMsg(pm.fromMsg(gripper) * self.palm)
        dx = p.position.x-real_tool[0][0]
        dy = p.position.y-real_tool[0][1]
        dz = p.position.z-real_tool[0][2]
        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        qw = quaternion_multiply(real_tool[1], quaternion_inverse([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))[3]
        if qw>0.99999:
            angle = 0
        elif qw<-0.99999:
             angle = numpy.pi
        else:
            angle = abs(2 * math.acos(qw))

        duration = length*20
        if angle*2.0>duration:
            duration = angle*2.0

        trj = CartesianTrajectory()
        trj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        
        trj.points.append(CartesianTrajectoryPoint(
        rospy.Duration(duration),
        p,
        Twist()))

        self.pub_trajectory.publish(trj)
        return duration

    def moveArm2(self, gripper_pose, duration):
        gripper = gripper_pose
        real_gripper = self.listener.lookupTransform('torso_base', self.prefix+'_HandPalmLink', rospy.Time(0))
        real_tool = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))
        p = pm.toMsg(pm.fromMsg(gripper) * self.palm)

        trj = CartesianTrajectory()
        trj.header.stamp = rospy.Time.now()
        
        trj.points.append(CartesianTrajectoryPoint(
        rospy.Duration(duration),
        p,
        Twist()))

        self.pub_trajectory.publish(trj)

    def stopArm(self):
        real_pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))

        trj = CartesianTrajectory()
        trj.header.stamp = rospy.Time.now()
        
#        trj.points.append(CartesianTrajectoryPoint(
#        rospy.Duration(1.0),
#        pm.toMsg(pm.fromTf(real_pose)),
#        Twist()))

        self.pub_trajectory.publish(trj)

    def move_hand_client(self, prefix, f1, f2, f3, spread):
        rospy.wait_for_service('/' + self.prefix + '_hand/move_hand')
        try:
            move_hand = rospy.ServiceProxy('/' + self.prefix + '_hand/move_hand', BHMoveHand)
            resp1 = move_hand(f1, f2, f3, spread, 0.7, 0.7, 0.7, 0.7, 1000, 1000, 1000, 1000)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def estCircle(self, px, py):
      x_m = mean(px)
      y_m = mean(py)
    
      def calc_R(xc, yc):
        """ calculate the distance of each 2D points from the center (xc, yc) """
        return sqrt((px-xc)**2 + (py-yc)**2)

      
      def f_2(c):
        """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
        Ri = calc_R(*c)
        return Ri - Ri.mean()
        
      center_estimate = x_m, y_m
      center_2, ier = optimize.leastsq(f_2, center_estimate)

      xc, yc = center_2
      Ri_2       = calc_R(xc, yc)
      R      = Ri_2.mean()
      return [xc, yc, R]
        
    def circle(self, cx, cy, r, a):
      dx = math.cos(a) * r
      dy = math.sin(a) * r
      px = cx + dx
      py = cy + dy
      return [px, py]
      
    def interpolate(begin, end, i, lenght):
      return begin + (((end - begin)/lenght)*i)  

    def publishDoorMarker(self, cx, cy, cz, r):
        m = MarkerArray()

        marker = Marker()
        marker.header.frame_id = 'torso_base'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'door'
        marker.id = 0
        marker.type = 3
        marker.action = 0
#        marker.points.append(Point(pressure_info.sensor[sens].center[i].x,pressure_info.sensor[sens].center[i].y,pressure_info.sensor[sens].center[i].z))
#        marker.points.append(Point(pressure_info.sensor[sens].center[i].x+cx, pressure_info.sensor[sens].center[i].y+cy, pressure_info.sensor[sens].center[i].z+cz))
        marker.pose.position.x = cx
        marker.pose.position.y = cy
        marker.pose.position.z = cz
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = r*2;
        marker.scale.y = r*2;
        marker.scale.z = 0.01;
        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        m.markers.append(marker)

        self.pub_marker.publish(m)

    def publishSinglePointMarker(self, x, y, z, i, r=0.0, g=1.0, b=0.0):
        m = MarkerArray()

        marker = Marker()
        marker.header.frame_id = 'torso_base'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'door'
        marker.id = i
        marker.type = 1
        marker.action = 0
#        marker.points.append(Point(pressure_info.sensor[sens].center[i].x,pressure_info.sensor[sens].center[i].y,pressure_info.sensor[sens].center[i].z))
#        marker.points.append(Point(pressure_info.sensor[sens].center[i].x+cx, pressure_info.sensor[sens].center[i].y+cy, pressure_info.sensor[sens].center[i].z+cz))
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.a = 0.5;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        m.markers.append(marker)

        self.pub_marker.publish(m)

    def hasContact(self, threshold):
        if self.max_tactile_value>threshold:
            return True
        return False

    def getContactPointOnFinger(self):
        pt = self.pressure_info.sensor[self.max_tactile_finger].center[self.max_tactile_index]
        return PyKDL.Vector(pt.x, pt.y, pt.z)
#        return PyKDL.Vector(0.04, -0.005, 0)
#        return PyKDL.Vector(0.0, 0.0, 0)

    def getContactPointFrame(self, contact_point_offset):
        self.listener.waitForTransform('/torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time.now(), rospy.Duration(4.0))
        pose_msg = self.listener.lookupTransform('/torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        return pm.fromTf(pose_msg)*PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), contact_point_offset)
#        return pm.fromTf(pose_msg)*PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.04, -0.005, 0))

    def openTheDoor(self):#, init_vector):
        px = []
        py = []

        self.listener.waitForTransform('/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', '/'+self.prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
        self.listener.waitForTransform('/torso_base', '/'+self.prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
        tool_msg = self.listener.lookupTransform('/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', '/'+self.prefix+'_HandPalmLink', rospy.Time(0))
        # end of the distal link of the middle finger
        contact_point_offset = self.getContactPointOnFinger()
        contact_point_frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), -contact_point_offset)
        tool = contact_point_frame*pm.fromTf(tool_msg)

        contact_frame = self.prefix+'_HandFingerThreeKnuckleThreeLink'

        rospy.sleep(1.0)
        
        init_motion = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0.25, -0.10))
        pose = self.getContactPointFrame(contact_point_offset)

        initial_contact_point = pm.toMsg(pose)
        p = pm.toMsg(pose * tool * init_motion)        
        px.append(initial_contact_point.position.x)
        py.append(initial_contact_point.position.y)


        self.listener.waitForTransform('torso_base', self.prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
        real_gripper = self.listener.lookupTransform('torso_base', self.prefix+'_HandPalmLink', rospy.Time(0))

        real_gripper_mx = quaternion_matrix(real_gripper[1])
        real_gripper_mx[:3, 3] = real_gripper[0][:3]
        grip_y = real_gripper_mx[:3,1]
        grip_z = real_gripper_mx[:3,2]
        init_motion_pt = -0.10*grip_z + [pm.toMsg(pose).position.x, pm.toMsg(pose).position.y, 0]

        pc = PointStamped()
        pc.header.frame_id = 'torso_base'
        pc.header.stamp = rospy.Time.now()
        pc.point = Point(pm.toMsg(pose).position.x, pm.toMsg(pose).position.y, 0)
        self.pub_pc.publish(pc)

        pc.header.stamp = rospy.Time.now()
#        pc.point = Point(pm.toMsg(pose).position.x + init_motion_pt[0], pm.toMsg(pose).position.y + init_motion_pt[1], 0)
        pc.point = Point(init_motion_pt[0], init_motion_pt[1], 0)
        self.pub_pc.publish(pc)


        pxm = PointStamped()
        pxm.header.frame_id = 'torso_base'

        print p

        # 7
        self.sendNextEvent()
        self.moveArm2(p, 3.0)

        self.br.sendTransform(self.PoseToTuple(p)[0], self.PoseToTuple(p)[1], rospy.Time.now(), "init_motion", "torso_base")
        print 'init motion'

        pt_prev = Point(initial_contact_point.position.x, initial_contact_point.position.y, 0)

        px.append(pt_prev.x)
        py.append(pt_prev.y)

        init_trajectory_progress = 0.0
        m_id = 1
        for i in range(0,120):

            rospy.sleep(0.025)
            contact_point_offset = self.getContactPointOnFinger()
            pose = self.getContactPointFrame(contact_point_offset)
            contact = pm.toMsg(pose)

            offset_y = (contact.position.x-initial_contact_point.position.x)*grip_y[0] + (contact.position.y-initial_contact_point.position.y)*grip_y[1]
            offset_z = -( (contact.position.x-initial_contact_point.position.x)*grip_z[0] + (contact.position.y-initial_contact_point.position.y)*grip_z[1] )
            print "o_z %s    o_y %s    a %s"%(offset_z,offset_y, 2.0*numpy.arctan2(offset_y,offset_z))

            dist = math.sqrt((pt_prev.x-contact.position.x)*(pt_prev.x-contact.position.x) + (pt_prev.y-contact.position.y)*(pt_prev.y-contact.position.y))
            if dist>0.005:
                px.append(contact.position.x)
                py.append(contact.position.y)
                pt_prev = Point(contact.position.x, contact.position.y, 0)
                self.publishSinglePointMarker(contact.position.x, contact.position.y, contact.position.z, m_id, 1.0, 0.0, 0.0)
                m_id += 1
                pxm.header.stamp = rospy.Time.now()
                pxm.point = Point(contact.position.x, contact.position.y, 0)
                self.pub_pm.publish(pxm)

            cz = contact.position.z
            if (i>20) and (not self.hasContact(50)):
                print "end: no contact"
                self.stopArm()
                return

#            if (offset_y>0.02) or (offset_y<-0.02):
#                self.stopArm()
#                break
            init_trajectory_progress += 1.0
        
        print "init motion finished"

        init_trajectory_progress /= 120.0
        if init_trajectory_progress<0.0:
            init_trajectory_progress = 0.0
        if init_trajectory_progress>1.0:
            init_trajectory_progress = 1.0

        print self.estCircle(px, py)
        cx, cy, r = self.estCircle(px, py)

        self.publishDoorMarker(cx, cy, cz, r)
        circle = QuaternionStamped()
        circle.header.frame_id = 'torso_base'
        circle.header.stamp = rospy.Time.now()
        circle.quaternion = Quaternion(cx, cy, cz, r)
        self.pub_circle.publish(circle)
        print "circle: x: %s   y: %s    r: %s"%(cx,cy,r)

        if r > 0.45 :
          return     
        if r < 0.10 :
          return
        r = r + 0.25
        # take angle form initial motion target point, not from the current position...
        a_init = math.atan2(initial_contact_point.position.y - cy, initial_contact_point.position.x - cx)
        a = math.atan2(init_motion_pt[1]*init_trajectory_progress + initial_contact_point.position.y*(1.0-init_trajectory_progress) - cy, init_motion_pt[0]*init_trajectory_progress + initial_contact_point.position.x*(1.0-init_trajectory_progress) - cx) + 0.01
#        a = math.atan2(contact.position.y - cy, contact.position.x - cx) #+ numpy.pi*10.0/180.0
        print a

        a_max = a_init + (numpy.pi*110.0/180.0)

        print "a_init: %s     a: %s     a_max: %s"%(a_init/numpy.pi*180.0, a/numpy.pi*180.0, a_max/numpy.pi*180.0)

        if rospy.is_shutdown():
            return
        trj_imp = CartesianImpedanceTrajectory()
        trj_imp.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        trj_imp.points.append(CartesianImpedanceTrajectoryPoint(
        rospy.Duration(3.0),
        CartesianImpedance(Wrench(Vector3(150.0, 35.0, 1000.0), Vector3(300.0, 300.0, 300.0)),Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
#        CartesianImpedance(Wrench(Vector3(75.0, 35.0, 1000.0), Vector3(300.0, 300.0, 300.0)),Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
        self.pub_impedance.publish(trj_imp)
        rospy.sleep(3.5)
        if rospy.is_shutdown():
            return

        raw_input("Press Enter to continue...")

        # 8
        self.sendNextEvent()
#        i = 1
        last_angle = a_init
        current_angle = a_init
        palm_rotation = 0.0
        while (a < a_max):
            if rospy.is_shutdown():
                break
            (x, y) = self.circle(cx, cy, r, a)

            pc.header.stamp = rospy.Time.now()
            pc.point = Point(x, y, 0)
            self.pub_pc.publish(pc)

            contact.position.x = x
            contact.position.y = y

            contact_point_offset = self.getContactPointOnFinger()
            contact_point_frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), -contact_point_offset)
            tool = contact_point_frame*pm.fromTf(tool_msg)

            frame = pm.fromMsg(contact)
            pt = pm.toMsg( frame * PyKDL.Frame(PyKDL.Rotation.RPY(0.0, 0.0, -palm_rotation), PyKDL.Vector(0.0, 0.0, 0.0)) * tool)
            self.publishSinglePointMarker(contact.position.x, contact.position.y, contact.position.z, m_id, 0.0, 1.0, 0.0)
            m_id += 1

            self.publishSinglePointMarker(pt.position.x, pt.position.y, pt.position.z, m_id, 0.0, 0.0, 1.0)
            m_id += 1

            self.moveArm2(pt, 1.0)
            self.br.sendTransform(self.PoseToTuple(pt)[0], self.PoseToTuple(pt)[1], rospy.Time.now(), 'init_motion_'+format(i), "torso_base")

            pose2 = self.getContactPointFrame(contact_point_offset)
            contact2 = pm.toMsg(pose2)

            dist = math.sqrt((pt_prev.x-contact2.position.x)*(pt_prev.x-contact2.position.x) + (pt_prev.y-contact2.position.y)*(pt_prev.y-contact2.position.y))
            current_angle = math.atan2(contact2.position.y - cy, contact2.position.x - cx)
            if (dist>0.005) and (current_angle>last_angle):
                px.append(contact2.position.x)
                py.append(contact2.position.y)
                pt_prev = Point(contact2.position.x, contact2.position.y, 0)
                self.publishSinglePointMarker(contact2.position.x, contact2.position.y, contact2.position.z, m_id, 1.0, 0.0, 0.0)
                m_id += 1
                pxm.header.stamp = rospy.Time.now()
                pxm.point = Point(contact2.position.x, contact2.position.y, 0)
                self.pub_pm.publish(pxm)
                last_angle = current_angle



#            print "a=%s    angle=%s     %s"%(a, 180.0*(-i * 0.01 - 0.1)/numpy.pi, self.estCircle(px, py))
            print "a=%s    angle=%s     %s"%(180.0*a/numpy.pi, 180.0*current_angle/numpy.pi, self.estCircle(px, py))
            cx, cy, r = self.estCircle(px, py)
            self.publishDoorMarker(cx, cy, cz, r)
            circle.header.stamp = rospy.Time.now()
            circle.quaternion = Quaternion(cx, cy, cz, r)
            self.pub_circle.publish(circle)

            if r > 0.50:
              break
            if r < 0.10 :
              break
            r = r + 0.25
            rospy.sleep(0.1)
            a = a + 0.01
#            i = i + 1

            if palm_rotation<(current_angle-a_init):
                palm_rotation += 0.08
            else:
                palm_rotation = (current_angle-a_init)

            if not self.hasContact(50):
                print "end: no contact"
                return


    def spin(self):

        # straighten fingers
        self.move_hand_client(self.prefix, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)

        rospy.sleep(1)
        if self.door_marker_visible:
            print "Found door marker"
        else:
            print "Could not find door marker"
            return

        # set impedence parameters
        duration = 3.0
        trj_imp = CartesianImpedanceTrajectory()
        trj_imp.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        trj_imp.points.append(CartesianImpedanceTrajectoryPoint(
        rospy.Duration(duration),
        CartesianImpedance(Wrench(Vector3(600.0, 1000.0, 1000.0), Vector3(300.0, 300.0, 300.0)),Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
        self.pub_impedance.publish(trj_imp)
        rospy.sleep(3.0)

        if rospy.is_shutdown():
            return

#        return

        # prepare to approach
        self.move_hand_client(self.prefix, 40.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)

        rospy.sleep(0.5)

        # save starting position
        self.listener.waitForTransform('torso_base', self.prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
        real_gripper = self.listener.lookupTransform('torso_base', self.prefix+'_HandPalmLink', rospy.Time(0))

        real_gripper_mx = quaternion_matrix(real_gripper[1])
        real_gripper_mx[:3, 3] = real_gripper[0][:3]

        # get door marker absolute position
        self.listener.waitForTransform('torso_base', 'ar_marker_3', rospy.Time.now(), rospy.Duration(4.0))
        door_marker = self.listener.lookupTransform('torso_base', 'ar_marker_3', rospy.Time(0))
        door_marker_mx = quaternion_matrix(door_marker[1])
        door_marker_mx[:3, 3] = door_marker[0][:3]


        marker_vertical_offset = -0.10
        initial_distance = 0.30
#        initial_distance_x = 0.08
        initial_distance_x = 0.0
        # G is the point where the center of gripper should be (near marker)
        G = door_marker[0] + initial_distance*door_marker_mx[:3,2] + (initial_distance_x)*door_marker_mx[:3,0] + marker_vertical_offset*door_marker_mx[:3,1]

        # x axis of the gripper is equal to inverted y axis of door's marker
        # y axis of the gripper is equal to inverted x axis of door's marker
        # z axis of the gripper is equal to inverted z axis of door's marker
#        if prefix == "right":
        grip_x = -door_marker_mx[:3,1]
        grip_y = -door_marker_mx[:3,0]
        grip_z = -door_marker_mx[:3,2]

        gripper_mx = identity_matrix()
        gripper_mx[:3,0] = grip_x
        gripper_mx[:3,1] = grip_y
        gripper_mx[:3,2] = grip_z
        gripper_mx[:3,3] = G

        # 0
        self.sendNextEvent()

        # move to pregrasp position
        self.br.sendTransform(translation_from_matrix(gripper_mx), quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
        duration = self.moveArm(self.tupleToPose([translation_from_matrix(gripper_mx),quaternion_from_matrix(gripper_mx)]))
        rospy.sleep(duration+0.5)

        rospy.sleep(2.0)

        if rospy.is_shutdown():
            return

        # 1
        self.sendNextEvent()

        contact_found = False
        # approach to door (along marker -z axis)
        distance = 0.0
        for i in range(1, 40):
            if rospy.is_shutdown():
                return
            distance += 0.005
#            print "distance: %s"%(initial_distance-distance)
            G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] + (initial_distance_x)*door_marker_mx[:3,0] + marker_vertical_offset*door_marker_mx[:3,1]
            self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
            duration = self.moveArm2(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]), 0.25)
#            rospy.sleep(duration+0.2)
            rospy.sleep(0.125)
#            print duration
            if self.hasContact(100):
                print "Found contact with door"
                contact_found = True
                break
            rospy.sleep(0.1)
            if self.hasContact(100):
                print "Found contact with door"
                contact_found = True
                break

        if not contact_found:
            return

        # 2
        self.sendNextEvent()

        # step back
        print "Going back 8cm"
        distance -= 0.08
        print "distance: %s"%(initial_distance-distance)
        G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] + (initial_distance_x)*door_marker_mx[:3,0] + marker_vertical_offset*door_marker_mx[:3,1]
        self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
        duration = self.moveArm(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]))
        rospy.sleep(duration+0.2)

        if rospy.is_shutdown():
            return

        # prepare to approach
        self.move_hand_client(self.prefix, 75.0/180.0*numpy.pi, 75.0/180.0*numpy.pi, 75.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)

        rospy.sleep(1.0)

        if rospy.is_shutdown():
            return

        # step forward
        print "Going forward 55cm"
        distance += 0.055
        print "distance: %s"%(initial_distance-distance)
        G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] + (initial_distance_x)*door_marker_mx[:3,0] + marker_vertical_offset*door_marker_mx[:3,1]
        self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
        duration = self.moveArm(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]))
        rospy.sleep(duration+0.2)

        # approach to handle (along marker -x axis)
        contact_found = False
        distance_x = 0.0
        for i in range(1, 40):
            if rospy.is_shutdown():
                return
            distance_x += 0.01
            print "distance_x: %s"%(distance_x)
            G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] + (initial_distance_x-distance_x)*door_marker_mx[:3,0] + marker_vertical_offset*door_marker_mx[:3,1]
            self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
            duration = self.moveArm2(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]), 0.5)
#            rospy.sleep(duration+0.2)
            rospy.sleep(0.25)
            if self.hasContact(100):
                print "Found contact with handle"
                contact_found = True
                break
            rospy.sleep(0.2)
            if self.hasContact(100):
                print "Found contact with handle"
                contact_found = True
                break

        if not contact_found:
            return

        # 3
        self.sendNextEvent()

        raw_input("Press Enter to continue...")

        # 4
        self.sendNextEvent()

        if rospy.is_shutdown():
            return
        trj_imp = CartesianImpedanceTrajectory()
        trj_imp.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        trj_imp.points.append(CartesianImpedanceTrajectoryPoint(
        rospy.Duration(3.0),
        CartesianImpedance(Wrench(Vector3(500.0, 35.0, 1000.0), Vector3(300.0, 300.0, 300.0)),Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
        self.pub_impedance.publish(trj_imp)
        rospy.sleep(3.5)
        if rospy.is_shutdown():
            return

        # 5
        self.sendNextEvent()

        distance_x += 0.22
        print "distance_x: %s"%(distance_x)
        G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] + (initial_distance_x-distance_x)*door_marker_mx[:3,0] + marker_vertical_offset*door_marker_mx[:3,1]
        self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
        duration = self.moveArm(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]))
        rospy.sleep(duration+0.1)

        # 6
        self.sendNextEvent()

        self.openTheDoor()
        return

        # return to pregrasp position
        self.br.sendTransform(translation_from_matrix(gripper_mx), quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
        duration = self.moveArm(self.tupleToPose([translation_from_matrix(gripper_mx),quaternion_from_matrix(gripper_mx)]))
        rospy.sleep(duration+0.5)

        # return to starting position
        duration = self.moveArm(self.tupleToPose([translation_from_matrix(real_gripper_mx),quaternion_from_matrix(real_gripper_mx)]))
        rospy.sleep(duration+0.5)

        # close all fingers
        self.move_hand_client(self.prefix, 120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)

        return

if __name__ == '__main__':
    rospy.init_node('door_opener')
    doorOpener = DoorOpener()

    try:
        doorOpener.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass

