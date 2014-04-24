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

    def alvarMarkerCallback(self, data):
        marker_count = len(data.markers)

        for i in range(0, marker_count):
            if data.markers[i].id == self.door_makrer_id:
                self.door_marker_pose = self.PoseToTuple(data.markers[i].pose.pose)
                self.door_marker_visible = True

    def tactileCallback(self, data):
        self.max_tip_f1 = max([data.finger1_tip[18], data.finger1_tip[19], data.finger1_tip[20], data.finger1_tip[21], data.finger1_tip[22], data.finger1_tip[23]])
        self.max_tip_f2 = max([data.finger2_tip[18], data.finger2_tip[19], data.finger2_tip[20], data.finger2_tip[21], data.finger2_tip[22], data.finger2_tip[23]])
        self.max_tip_f3 = max([data.finger3_tip[18], data.finger3_tip[19], data.finger3_tip[20], data.finger3_tip[21], data.finger3_tip[22], data.finger3_tip[23]])

        self.max_distal_f1 = max(data.finger1_tip)
        self.max_distal_f2 = max(data.finger2_tip)
        self.max_distal_f3 = max(data.finger3_tip)

    def __init__(self):
        self.frame_id = 'base_footprint'
        self.prev_time = rospy.Time.now()

        # publisher with imu data
        self.pub_trajectory = rospy.Publisher("/right_arm/trajectory", CartesianTrajectory)
        self.pub_impedance = rospy.Publisher("/right_arm/impedance", CartesianImpedanceTrajectory)
        self.listener = tf.TransformListener();
        self.br = tf.TransformBroadcaster()

        self.pub_marker = rospy.Publisher('/door_markers', MarkerArray)

        rospy.sleep(1.0)
        
        self.door_makrer_id=3
        self.prefix="right"

        palm_msg = self.listener.lookupTransform(self.prefix+'_HandPalmLink', self.prefix+'_arm_7_link', rospy.Time(0))
        self.palm = pm.fromTf(palm_msg)
        self.door_marker_visible = False
        self.door_marker_pose = Pose()

        self.max_tip_f1 = 0
        self.max_tip_f2 = 0
        self.max_tip_f3 = 0
        self.max_distal_f1 = 0
        self.max_distal_f2 = 0
        self.max_distal_f3 = 0

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
            qw=0.99999
        if qw<-0.99999:
            qw=-0.99999
        angle = abs(2 * math.acos(qw))
        duration = length*20
        if angle*2>duration:
            duration = angle*2

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
# + rospy.Duration(0.1)
        
        trj.points.append(CartesianTrajectoryPoint(
        rospy.Duration(duration),
        p,
        Twist()))

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
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 0.5;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        m.markers.append(marker)

        self.pub_marker.publish(m)

    def getContactPointFrame(self):
        self.listener.waitForTransform('/torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time.now(), rospy.Duration(4.0))
        pose_msg = self.listener.lookupTransform('/torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        return pm.fromTf(pose_msg)*PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.04, -0.005, 0))

    def openTheDoor(self):
        px = []
        py = []
        
#        pxc = []
#        pyc = []

        contact_point_offset = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(-0.04, 0.005, 0))
        self.listener.waitForTransform('/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', '/'+self.prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
        self.listener.waitForTransform('/torso_base', '/'+self.prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
        tool_msg = self.listener.lookupTransform('/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', '/'+self.prefix+'_HandPalmLink', rospy.Time(0))
        # end of the distal link of the middle finger
        tool = contact_point_offset*pm.fromTf(tool_msg)

        contact_frame = self.prefix+'_HandFingerThreeKnuckleThreeLink'

        self.prev_time = rospy.Time.now()

        rospy.sleep(1.0)
        
#        pose_msg = self.listener.lookupTransform('/torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        init_motion = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0.0, -0.10))
        pose = self.getContactPointFrame()
#        p_contact = pm.toMsg(pose) 
#        p_before = pm.toMsg(pose * tool)

        p = pm.toMsg(pose * tool * init_motion)        
#        p = pm.toMsg(pm.fromTf(pose_msg) * tool * init_motion)
#        p = pm.toMsg(init_motion * tool)
#        p1 = pm.toMsg(pm.fromTf(pose_msg) * init_motion)
        
#        print pose_msg
        print p

        self.moveArm2(p, 3.0)

        self.br.sendTransform(self.PoseToTuple(p)[0], self.PoseToTuple(p)[1], rospy.Time.now(), "init_motion", "torso_base")
#        self.br.sendTransform(self.PoseToTuple(p_contact)[0], self.PoseToTuple(p_contact)[1], rospy.Time.now(), "init_motion1", "torso_base")
#        self.br.sendTransform(self.PoseToTuple(p_before)[0], self.PoseToTuple(p_before)[1], rospy.Time.now(), "init_motion2", "torso_base")
        print 'init motion'

        m_id = 1
        rospy.sleep(0.3)
        for i in range(0,11):
          rospy.sleep(0.2)
#          pose_msg = self.listener.lookupTransform('/torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
#          pose_msg_mx = quaternion_matrix(pose_msg[1])
#          pose_msg_mx[:3, 3] = pose_msg[0][:3]
#          contact = numpy.dot(pose_msg_mx, [0.04, 0.0, 0.0, 1.0])
          pose = self.getContactPointFrame()
          contact = pm.toMsg(pose)
          px.append(contact.position.x)
          py.append(contact.position.y)
          cz = contact.position.z
          self.publishSinglePointMarker(contact.position.x, contact.position.y, cz, m_id, 1.0, 0.0, 0.0)
          m_id += 1
#          self.br.sendTransform(translation_from_matrix(pose_msg_mx), quaternion_from_matrix(pose_msg_mx), rospy.Time.now(), "test1", "torso_base")
        
        print self.estCircle(px, py)
        cx, cy, r = self.estCircle(px, py)

        self.publishDoorMarker(cx, cy, cz, r)
        print "circle: x: %s   y: %s    r: %s"%(cx,cy,r)

#        return

        r = r + 0.01
        if r > 0.45 :
          return     
        if r < 0.15 :
          return     
        a = math.atan2(contact.position.y - cy, contact.position.x - cx) + 0.01
        print a
        #raw_input("Press Enter to continue...")

        a_max = a + (numpy.pi*60.0/180.0)

        i = 1
        while (a < a_max):
          if rospy.is_shutdown():
              break
          (x, y) = self.circle(cx, cy, r, a)

          contact.position.x = x
          contact.position.y = y

          frame = pm.fromMsg(contact)
#          pt = pm.toMsg(frame * tool * PyKDL.Frame(PyKDL.Rotation.RPY(-i * 0.01 - 0.1, 0.0, 0.0), PyKDL.Vector(0.0, 0.0, 0.0)))
          pt = pm.toMsg( frame * PyKDL.Frame(PyKDL.Rotation.RPY(0.0, 0.0, -i * 0.01 - 0.1), PyKDL.Vector(0.0, 0.0, 0.0)) * tool)
          self.publishSinglePointMarker(contact.position.x, contact.position.y, contact.position.z, m_id, 0.0, 1.0, 0.0)
          m_id += 1

          self.moveArm2(pt, 1.0)
          self.br.sendTransform(self.PoseToTuple(pt)[0], self.PoseToTuple(pt)[1], rospy.Time.now(), 'init_motion_'+format(i), "torso_base")

          pose2 = self.getContactPointFrame()
          contact2 = pm.toMsg(pose2)
          px.append(contact2.position.x)
          py.append(contact2.position.y)

          self.publishSinglePointMarker(contact2.position.x, contact2.position.y, contact2.position.z, m_id, 1.0, 0.0, 0.0)
          m_id += 1
          
          print "a=%s    angle=%s     %s"%(a, 180.0*(-i * 0.01 - 0.1)/numpy.pi, self.estCircle(px, py))
          cx, cy, r = self.estCircle(px, py)
          self.publishDoorMarker(cx, cy, cz, r)

          r = r + 0.04
          
          if r > 0.45:
            break
          if r < 0.15 :
            break
          rospy.sleep(0.1)
          a = a + 0.01
          i = i + 1
        
        #print [px, py]
#        sio.savemat('data.mat', {'Tm':[px, py], 'Tc':[pxc, pyc]}, oned_as='row')

    def spin(self):


#        raw_input("Press Enter to continue...")
#        rospy.sleep(3)
#        self.openTheDoor()

#        return

        # find the door handle

        # straighten fingers
        self.move_hand_client(self.prefix, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
#        self.move_hand_client(self.prefix, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi)

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
        CartesianImpedance(Wrench(Vector3(600.0, 600.0, 1000.0), Vector3(300.0, 300.0, 300.0)),Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
        self.pub_impedance.publish(trj_imp)
        rospy.sleep(3.0)

        if rospy.is_shutdown():
            return

        # prepare to approach
        self.move_hand_client(self.prefix, 40.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
#        self.move_hand_client(self.prefix, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 0.0/180.0*numpy.pi)

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


        marker_vertical_offset = -0.8
        initial_distance = 0.30
        # G is the point where the center of gripper should be (near marker)
        G = door_marker[0] + initial_distance*door_marker_mx[:3,2] + marker_vertical_offset*door_marker_mx[:3,1]

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

        # move to pregrasp position
        self.br.sendTransform(translation_from_matrix(gripper_mx), quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
        duration = self.moveArm(self.tupleToPose([translation_from_matrix(gripper_mx),quaternion_from_matrix(gripper_mx)]))
        rospy.sleep(duration+0.5)

        rospy.sleep(2.0)

        if rospy.is_shutdown():
            return

        # approach to door (along marker -z axis)
        distance = 0.0
        for i in range(1, 10):
            if rospy.is_shutdown():
                return
            distance += 0.02
            print "distance: %s"%(initial_distance-distance)
            G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] + marker_vertical_offset*door_marker_mx[:3,1]
            self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
            duration = self.moveArm(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]))
            rospy.sleep(duration+0.2)
            if (self.max_tip_f1>100) or (self.max_tip_f2>100) or (self.max_tip_f3>100):
                print "Found contact with door"
                break

        # step back
        print "Going back 8cm"
        distance -= 0.08
        print "distance: %s"%(initial_distance-distance)
        G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] + marker_vertical_offset*door_marker_mx[:3,1]
        self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
        duration = self.moveArm(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]))
        rospy.sleep(duration+0.2)

        if rospy.is_shutdown():
            return

        # prepare to approach
        self.move_hand_client(self.prefix, 75.0/180.0*numpy.pi, 75.0/180.0*numpy.pi, 75.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
#        self.move_hand_client(self.prefix, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 80.0/180.0*numpy.pi, 0.0/180.0*numpy.pi)

        rospy.sleep(1.0)

        if rospy.is_shutdown():
            return

        # step forward
        print "Going forward 5cm"
        distance += 0.05
        print "distance: %s"%(initial_distance-distance)
        G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] + marker_vertical_offset*door_marker_mx[:3,1]
        self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
        duration = self.moveArm(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]))
        rospy.sleep(duration+0.2)

        # approach to handle (along marker -x axis)
        distance_x = 0.0
        for i in range(1, 20):
            if rospy.is_shutdown():
                return
            distance_x += 0.02
            print "distance_x: %s"%(distance_x)
            G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] - distance_x*door_marker_mx[:3,0] + marker_vertical_offset*door_marker_mx[:3,1]
            self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
            duration = self.moveArm(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]))
            rospy.sleep(duration+0.2)
            if (self.max_distal_f1>100) or (self.max_distal_f2>100) or (self.max_distal_f3>100):
                print "Found contact with handle"
                break

        # step back
#        distance_x -= 0.03
#        print "distance_x: %s"%(distance_x)
#        G_approach = door_marker[0] + (initial_distance-distance)*door_marker_mx[:3,2] - distance_x*door_marker_mx[:3,0] + marker_vertical_offset*door_marker_mx[:3,1]
#        self.br.sendTransform(G_approach, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "door", "torso_base")
#        duration = self.moveArm(self.tupleToPose([G_approach,quaternion_from_matrix(gripper_mx)]))
#        rospy.sleep(duration+0.2)

        raw_input("Press Enter to continue...")
        if rospy.is_shutdown():
            return

        # close fingers a bit more
#        self.move_hand_client(self.prefix, 95.0/180.0*numpy.pi, 95.0/180.0*numpy.pi, 95.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)

        trj_imp = CartesianImpedanceTrajectory()
        trj_imp.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        trj_imp.points.append(CartesianImpedanceTrajectoryPoint(
        rospy.Duration(3.0),
        CartesianImpedance(Wrench(Vector3(1000.0, 300.0, 1000.0), Vector3(300.0, 300.0, 300.0)),Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
        self.pub_impedance.publish(trj_imp)
        rospy.sleep(3.5)
        if rospy.is_shutdown():
            return

#        rospy.sleep(3.0)
        self.openTheDoor()

        return
#        rospy.sleep(15.0)

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

