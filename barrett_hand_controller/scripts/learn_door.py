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
import actionlib
from actionlib_msgs.msg import *

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

import copy

# reference frames:
# B - robot's base
# R - camera
# W - wrist
# E - gripper
# F - finger distal link
# T - tool
# C - current contact point
# N - the end point of finger's nail
# M - door marker frame (door base)

class Attribute:
    def __init__(self):
        self.name = None

class PoseAttribute(Attribute):
    def __init__(self, name, value = None):
        self.name = name
        self.value = value

class ScalarAttribute(Attribute):
    def __init__(self, name, value = None):
        self.name = name
        self.value = value

class Door:
    def __init__(self):
        self.attributes = []
        self.attributes.append( PoseAttribute("base", None) )         # door base frame, constant, not moving
        # points for better surface estimation by touch
        self.attributes.append( PoseAttribute("P1_start", None) )     # pose on door surface
        self.attributes.append( PoseAttribute("P1_end", None) )       # pose on door surface
        self.attributes.append( PoseAttribute("P2_start", None) )     # pose on door surface
        self.attributes.append( PoseAttribute("P2_end", None) )       # pose on door surface
        self.attributes.append( PoseAttribute("P3_start", None) )     # pose on door surface
        self.attributes.append( PoseAttribute("P3_end", None) )       # pose on door surface
        self.attributes.append( PoseAttribute("H_start", None) )      # handle search
        self.attributes.append( PoseAttribute("H_end", None) )        # handle search
        self.attributes.append( PoseAttribute("hinge", None) )        # hinge frame, in base frame, constant
        self.attributes.append( ScalarAttribute("handle_radius", None) )
        self.attributes.append( PoseAttribute("handle", None) )
        self.attributes.append( PoseAttribute("pre_handle", None) )

    def getAttribute(self, name):
        return next(a for a in self.attributes if a.name == name)

    def printAttributes(self):
        for a in self.attributes:
            print "name: %s"%(a.name)
            print "value:"
            print a.value

class DoorOpener:
    """
Class for opening door with velma robot.
"""
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
        if self.tactile_force_min > self.max_tactile_value:
            self.tactile_force_min = self.max_tactile_value

    def sendNextEvent(self):
        pc = PointStamped()
        pc.header.frame_id = 'torso_base'
        pc.header.stamp = rospy.Time.now()
        pc.point = Point(self.pub_msg_id, 0, 0)
        self.pub_msg.publish(pc)
        self.pub_msg_id += 1

    def getMaxWrench(self):
        wrench = Wrench()
        for i in range(0,self.wrench_tab_len):
            if abs(self.wrench_tab[i].force.x) > wrench.force.x:
                wrench.force.x = abs(self.wrench_tab[i].force.x)
            if abs(self.wrench_tab[i].force.y) > wrench.force.y:
                wrench.force.y = abs(self.wrench_tab[i].force.y)
            if abs(self.wrench_tab[i].force.z) > wrench.force.z:
                wrench.force.z = abs(self.wrench_tab[i].force.z)
            if abs(self.wrench_tab[i].torque.x) > wrench.torque.x:
                wrench.torque.x = abs(self.wrench_tab[i].torque.x)
            if abs(self.wrench_tab[i].torque.y) > wrench.torque.y:
                wrench.torque.y = abs(self.wrench_tab[i].torque.y)
            if abs(self.wrench_tab[i].torque.z) > wrench.torque.z:
                wrench.torque.z = abs(self.wrench_tab[i].torque.z)
        return wrench

    def wrenchCallback(self, wrench):
        self.wrench_tab[self.wrench_tab_index] = wrench
        self.wrench_tab_index += 1
        if self.wrench_tab_index >= self.wrench_tab_len:
            self.wrench_tab_index = 0
        wfx = abs(wrench.force.x)
        wfy = abs(wrench.force.y)
        wfz = abs(wrench.force.z)
        wtx = abs(wrench.torque.x)
        wty = abs(wrench.torque.y)
        wtz = abs(wrench.torque.z)
        self.wrench_mean.force.x = self.wrench_mean.force.x * self.wrench_mean_count + wfx
        self.wrench_mean.force.y = self.wrench_mean.force.y * self.wrench_mean_count + wfy
        self.wrench_mean.force.z = self.wrench_mean.force.z * self.wrench_mean_count + wfz
        self.wrench_mean.torque.x = self.wrench_mean.torque.x * self.wrench_mean_count + wtx
        self.wrench_mean.torque.y = self.wrench_mean.torque.y * self.wrench_mean_count + wty
        self.wrench_mean.torque.z = self.wrench_mean.torque.z * self.wrench_mean_count + wtz
        self.wrench_mean_count += 1
        self.wrench_mean.force.x /= self.wrench_mean_count
        self.wrench_mean.force.y /= self.wrench_mean_count
        self.wrench_mean.force.z /= self.wrench_mean_count
        self.wrench_mean.torque.x /= self.wrench_mean_count
        self.wrench_mean.torque.y /= self.wrench_mean_count
        self.wrench_mean.torque.z /= self.wrench_mean_count
        if self.wrench_max.force.x < wfx:
            self.wrench_max.force.x = wfx
        if self.wrench_max.force.y < wfy:
            self.wrench_max.force.y = wfy
        if self.wrench_max.force.z < wfz:
            self.wrench_max.force.z = wfz
        if self.wrench_max.torque.x < wtx:
            self.wrench_max.torque.x = wtx
        if self.wrench_max.torque.y < wty:
            self.wrench_max.torque.y = wty
        if self.wrench_max.torque.z < wtz:
            self.wrench_max.torque.z = wtz
        if (wfx>self.current_max_wrench.force.x*2.0) or (wfy>self.current_max_wrench.force.y*2.0) or (wfz>self.current_max_wrench.force.z*2.0) or (wtx>self.current_max_wrench.torque.x*2.0) or (wty>self.current_max_wrench.torque.y*2.0) or (wtz>self.current_max_wrench.torque.z*2.0):
            self.wrench_emergency_stop = True

    def resetMarkCounters(self):
        self.wrench_max = Wrench()
        self.wrench_mean_count = 0
        self.wrench_mean = Wrench()
        self.tactile_force_min = 1000000.0

    def __init__(self):
        # parameters
        self.prefix="right"
        self.q_start = (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 180.0/180.0*numpy.pi) 
        self.q_door = (120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
        self.q_handle = (75.0/180.0*numpy.pi, 75.0/180.0*numpy.pi, 75.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
        self.q_close = (120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
        self.d_init = 0.1
        self.delta = 0.02
        self.delta_e = 0.05
        self.k_door = Wrench(Vector3(200.0, 800.0, 800.0), Vector3(300.0, 300.0, 300.0))
        self.k_door2 = Wrench(Vector3(800.0, 800.0, 800.0), Vector3(300.0, 300.0, 300.0))
        self.k_error = Wrench(Vector3(10.0, 10.0, 10.0), Vector3(2.0, 2.0, 2.0))
        self.k_close = Wrench(Vector3(400.0, 400.0, 400.0), Vector3(100.0, 100.0, 100.0))
        self.T_W_T = PyKDL.Frame(PyKDL.Vector(0.2,-0.05,0))    # tool transformation
        self.T_W_E = None
        self.T_E_W = None
        self.T_F_N = PyKDL.Frame( PyKDL.Vector(0.05, 0.01, 0) )
        self.T_N_F = self.T_F_N.Inverse()
        self.current_max_wrench = Wrench(Vector3(20, 20, 20), Vector3(20, 20, 20))
        self.wrench_emergency_stop = False
        self.exit_on_emergency_stop = True

        self.init_motion_time = 3.0
        self.door_angle_dest = 80.0/180.0*math.pi

        # for score function
        self.init_motion_time_left = 3.0
        self.circle_cx = 0
        self.circle_cy = 0
        self.circle_r = 0
        self.failure_reason = "unknown"

        self.emergency_stop_active = False

        self.resetMarkCounters()

        self.action_trajectory_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/cartesian_trajectory", CartesianTrajectoryAction)
        self.action_trajectory_client.wait_for_server()

        self.action_tool_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/tool_trajectory", CartesianTrajectoryAction)
        self.action_tool_client.wait_for_server()

        self.action_impedance_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/cartesian_impedance", CartesianImpedanceAction)
        self.action_impedance_client.wait_for_server()

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

        self.door_marker_visible = False

        self.max_tactile_value = 0
        self.max_tactile_index = 0
        self.max_tactile_finger = 0

        print "Requesting pressure sensors info"
        self.pressure_info = self.get_pressure_sensors_info_client()

        self.wrench_tab = []
        self.wrench_tab_index = 0
        self.wrench_tab_len = 4000
        for i in range(0,self.wrench_tab_len):
            self.wrench_tab.append( Wrench(Vector3(), Vector3()) )

        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvarMarkerCallback)
        rospy.Subscriber('/'+self.prefix+'_hand/BHPressureState', BHPressureState, self.tactileCallback)
        rospy.Subscriber('/'+self.prefix+'_arm/wrench', Wrench, self.wrenchCallback)

    def moveWrist2(self, wrist_frame):
        wrist_pose = pm.toMsg(wrist_frame)#*self.T_W_T)
        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), "dest", "torso_base")

    def moveWrist(self, wrist_frame, t, max_wrench, start_time=0.01, stamp=None):
        # we are moving the tool, so: T_B_Wd*T_W_T
        wrist_pose = pm.toMsg(wrist_frame)
        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), "dest", "torso_base")
        wrist_pose = pm.toMsg(wrist_frame*self.T_W_T)

        action_trajectory_goal = CartesianTrajectoryGoal()
        if stamp != None:
            action_trajectory_goal.trajectory.header.stamp = stamp
        else:
            action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(t),
        wrist_pose,
        Twist()))
        action_trajectory_goal.wrench_constraint = max_wrench
        self.current_max_wrench = max_wrench
        self.action_trajectory_client.send_goal(action_trajectory_goal)

    def moveWristTraj(self, wrist_frames, times, max_wrench, start_time=0.01, stamp=None):
        # we are moving the tool, so: T_B_Wd*T_W_T
        action_trajectory_goal = CartesianTrajectoryGoal()
        if stamp != None:
            action_trajectory_goal.trajectory.header.stamp = stamp
        else:
            action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)

        i = 0
        for wrist_frame in wrist_frames:
            wrist_pose = pm.toMsg(wrist_frame*self.T_W_T)
            action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
            rospy.Duration(times[i]),
            wrist_pose,
            Twist()))
            i += 1

        action_trajectory_goal.wrench_constraint = max_wrench
        self.current_max_wrench = max_wrench
        self.action_trajectory_client.send_goal(action_trajectory_goal)

    def moveTool(self, wrist_frame, t):
        wrist_pose = pm.toMsg(wrist_frame)

        action_tool_goal = CartesianTrajectoryGoal()
        action_tool_goal.trajectory.header.stamp = rospy.Time.now()
        action_tool_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(t),
        wrist_pose,
        Twist()))
        self.action_tool_client.send_goal(action_tool_goal)

    def moveImpedance(self, k, t, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        self.action_impedance_goal = CartesianImpedanceGoal()
        if stamp != None:
            self.action_impedance_goal.trajectory.header.stamp = stamp
        else:
            self.action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        self.action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
        rospy.Duration(t),
        CartesianImpedance(k,damping)))
        self.action_impedance_client.send_goal(self.action_impedance_goal)

    def moveImpedanceTraj(self, k_n, t_n, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        self.action_impedance_goal = CartesianImpedanceGoal()
        if stamp != None:
            self.action_impedance_goal.trajectory.header.stamp = stamp
        else:
            self.action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        i = 0
        for k in k_n:
            self.action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
            rospy.Duration(t_n[i]),
            CartesianImpedance(k,damping)))

        self.action_impedance_client.send_goal(self.action_impedance_goal)

    def stopArm(self):
        if self.action_trajectory_client.gh:
            self.action_trajectory_client.cancel_all_goals()
        if self.action_tool_client.gh:
            self.action_tool_client.cancel_all_goals()

    def emergencyStop(self):
        self.moveImpedance(self.k_error, 0.5)
        self.stopArm()
        self.emergency_stop_active = True
        print "emergency stop"

    def emergencyExit(self):
        exit(0)

    def checkStopCondition(self, t=0.0):

        end_t = rospy.Time.now()+rospy.Duration(t+0.0001)
        while rospy.Time.now()<end_t:
            if rospy.is_shutdown():
                self.emergencyStop()
                print "emergency stop: interrupted  %s  %s"%(self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "user_interrupt"
                rospy.sleep(1.0)
                if self.exit_on_emergency_stop:
                    self.emergencyExit()
            if self.wrench_emergency_stop:
                self.emergencyStop()
                print "too big wrench"
                self.failure_reason = "too_big_wrench"
                rospy.sleep(1.0)
                if self.exit_on_emergency_stop:
                    self.emergencyExit()

            if (self.action_trajectory_client.gh) and ((self.action_trajectory_client.get_state()==GoalStatus.REJECTED) or (self.action_trajectory_client.get_state()==GoalStatus.ABORTED)):
                state = self.action_trajectory_client.get_state()
                result = self.action_trajectory_client.get_result()
                self.emergencyStop()
                print "emergency stop: traj_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "too_big_wrench_trajectory"
                rospy.sleep(1.0)
                if self.exit_on_emergency_stop:
                    self.emergencyExit()

            if (self.action_tool_client.gh) and ((self.action_tool_client.get_state()==GoalStatus.REJECTED) or (self.action_tool_client.get_state()==GoalStatus.ABORTED)):
                state = self.action_tool_client.get_state()
                result = self.action_tool_client.get_result()
                self.emergencyStop()
                print "emergency stop: tool_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "too_big_wrench_tool"
                rospy.sleep(1.0)
                if self.exit_on_emergency_stop:
                    self.emergencyExit()
            rospy.sleep(0.01)
        return self.emergency_stop_active


    def move_hand_client(self, prefix, q):
        rospy.wait_for_service('/' + self.prefix + '_hand/move_hand')
        try:
            move_hand = rospy.ServiceProxy('/' + self.prefix + '_hand/move_hand', BHMoveHand)
            resp1 = move_hand(q[0], q[1], q[2], q[3], 1.2, 1.2, 1.2, 1.2, 2000, 2000, 2000, 2000)
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
      center_2, ier = optimize.leastsq(f_2, center_estimate, maxfev = 3000)

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
        marker.pose = Pose( Point(cx,cy,cz), Quaternion(0,0,0,1) )
        marker.scale = Vector3(r*2.0, r*2.0, 0.01)
        marker.color = ColorRGBA(1,0,0,0.5)
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
        marker.pose = Pose( Point(x,y,z), Quaternion(0,0,0,1) )
        marker.scale = Vector3(0.005, 0.005, 0.005)
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)

        self.pub_marker.publish(m)

    def hasContact(self, threshold):
        if self.max_tactile_value>threshold:
            return True
        return False

    def getContactPointFrame(self):
        pt = self.pressure_info.sensor[self.max_tactile_finger].center[self.max_tactile_index]
        T_F_C = PyKDL.Frame(PyKDL.Vector(pt.x, pt.y, pt.z))
        return T_F_C

    def getTransformations(self):
        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))
        self.T_B_W = pm.fromTf(pose)

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        self.T_E_F = pm.fromTf(pose)
        self.T_F_E = self.T_E_F.Inverse()

        self.T_F_C = self.getContactPointFrame()
        self.T_C_F = self.T_F_C.Inverse()

        if self.T_W_E == None:
            pose = self.listener.lookupTransform(self.prefix+'_arm_7_link', self.prefix+'_HandPalmLink', rospy.Time(0))
            self.T_W_E = pm.fromTf(pose)
            self.T_E_W = self.T_W_E.Inverse()

    def stepBackFromHandle(self):
        self.getTransformations()
        print "moving desired pose to current pose"
        self.moveWrist(self.T_B_W, 2.0, Wrench(Vector3(20,20,20), Vector3(6,4,2)))
        self.checkStopCondition(2.0)

        self.getTransformations()
        print "moving gripper outwards the handle"
        T_W_Wd = PyKDL.Frame(PyKDL.Vector(0,-0.05,0))
        T_B_Wd = self.T_B_W*T_W_Wd
        self.moveWrist(T_B_Wd, 0.5, Wrench(Vector3(10,15,10), Vector3(4,3,2)))
        self.checkStopCondition(0.5)

        self.getTransformations()
        print "rotating gripper outwards the handle"
        T_W_Wd = PyKDL.Frame(PyKDL.Rotation.RotZ(-math.pi/8.0))
        T_B_Wd = self.T_B_W*T_W_Wd
        self.moveWrist(T_B_Wd, 2.0, Wrench(Vector3(10,15,10), Vector3(4,3,2)))
        self.checkStopCondition(2.0)

    def addContact(self, P_contact):
        if self.hasContact(50):
            if len(self.px) > 0:
                dist = math.sqrt((self.px[-1]-P_contact.x())*(self.px[-1]-P_contact.x()) + (self.py[-1]-P_contact.y())*(self.py[-1]-P_contact.y()))
                if dist > 0.005:
                    self.px.append(P_contact.x())
                    self.py.append(P_contact.y())
                    self.publishSinglePointMarker(P_contact.x(), P_contact.y(), P_contact.z(), self.m_id, 1.0, 0.0, 0.0)
                    self.m_id += 1
            else:
                self.px.append(P_contact.x())
                self.py.append(P_contact.y())

    def calculateMoveGripperPointToPose(self, P_E_p, rot_B_E, pos_B_E_p):
        P_B_N = PyKDL.Frame( copy.deepcopy(rot_B_E) ) * P_E_p
        print P_B_N
        T_B_Ed = PyKDL.Frame( copy.deepcopy(rot_B_E), pos_B_E_p - P_B_N )
        T_B_Wd = T_B_Ed * self.T_E_W
        return T_B_Wd

    def openTheDoor(self, R_B_E):
        self.px = []
        self.py = []
        self.m_id = 0

        #
        # score setup
        #
        self.score_open_A = 1000.0   # at init_motion_time_left == init_motion_time
        self.score_open_B = 800.0    # at init_motion_time_left == 0
        self.score_open_C = 700.0    # when gripper orientation is reached
        self.score_open_D = 0.0    # when door_angle == door_angle_dest

        self.resetMarkCounters()

        #
        # calculate desired point of contact (middle of tactile sensor) in E frame
        #
        pt = self.pressure_info.sensor[2].center[10]
        T_F_pt = PyKDL.Frame(PyKDL.Vector(pt.x, pt.y, pt.z))
        E_pt = self.T_E_F * T_F_pt * PyKDL.Vector(0,0,0)

        #
        # initial motion setup
        #
        self.getTransformations()

        T_E_Ed = PyKDL.Frame(PyKDL.Vector(0, self.r_a, -self.d_init))
        T_W_B = self.T_B_W.Inverse()

        T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
        P_contact = T_B_C*PyKDL.Vector(0,0,0)
        self.addContact(P_contact)
        pz = P_contact.z()
        T_B_Wd_init = self.T_B_W * self.T_W_E * T_E_Ed * self.T_E_W

        #
        # initial motion begins
        #
        self.moveWrist(T_B_Wd_init, self.init_motion_time, Wrench(Vector3(10,25,25), Vector3(4,3,2)))

        lost_contact = False
        init_end_t = rospy.Time.now()+rospy.Duration(self.init_motion_time)
        init_contact_measure_t = rospy.Time.now()+rospy.Duration(1.0)
        while rospy.Time.now()<init_end_t:
            self.getTransformations()
            T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
            P_contact = T_B_C*PyKDL.Vector(0,0,0)
            self.addContact(P_contact)

            if (rospy.Time.now() > init_contact_measure_t) and (not self.hasContact(50)):
                lost_contact = True

            if lost_contact or self.checkStopCondition(0.01):
                # for score function
                dur = init_end_t-rospy.Time.now()
                self.init_motion_time_left = dur.to_sec()
                f = self.init_motion_time_left / self.init_motion_time
                score = f*self.score_open_A + (1.0-f)*self.score_open_B
                self.emergencyStop()
                if lost_contact:
                    self.failure_reason = "lost_contact"
                    print "end: lost contact"
                rospy.sleep(1.0)
                return score

        print "init motion finished"

        #
        # first circle estimation
        #
        if len(self.px)>1:
            cx, cy, r = self.estCircle(self.px, self.py)
            r_old = r
        else:
            self.failure_reason = "no_contact_points"
            print "end: no contact points for estimation"
            return self.score_open_B

        # for score function
        self.init_motion_time_left = 0.0
        self.circle_cx = cx
        self.circle_cy = cy
        self.circle_r = r

        self.publishDoorMarker(cx, cy, pz, r)
        circle = QuaternionStamped()
        circle.header.frame_id = 'torso_base'
        circle.header.stamp = rospy.Time.now()
        circle.quaternion = Quaternion(cx, cy, pz, r)
        self.pub_circle.publish(circle)
        print "circle: x: %s   y: %s    r: %s"%(cx,cy,r)

        # vector from initial contact point to estimated circle center
        V_p_c = PyKDL.Vector(cx-self.px[0], cy-self.py[0], 0)
        # vector pointing to the right side of the door marker
        T_B_M = door.getAttribute("base").value
        V_right = T_B_M*PyKDL.Vector(1,0,0)

        # for right door the dot product should be positive
        if PyKDL.dot(V_p_c, V_right) < 0:
            self.failure_reason = "estimation_error_1"
            print "estimation error: door is not right"
            return self.score_open_B
        if r > 0.45 :
            self.failure_reason = "estimation_error_2"
            print "estimation error: too big radius"
            return self.score_open_B
        if r < 0.075 :
            self.failure_reason = "estimation_error_3"
            print "estimation error: too small radius"
            return self.score_open_B

        #
        # rotate the gripper and destination point around contact point
        #
        # calculate absolute door angle just after the first contact between handle and gripper
        alpha_init = math.atan2( self.py[0] - cy, self.px[0] - cx )
        # calculate destination angle

        self.checkStopCondition(0.5)

        # get current contact point
        self.getTransformations()
        T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
        P_contact = T_B_C*PyKDL.Vector(0,0,0)
        # calculate current door angle
        alpha_door = math.atan2(P_contact.y()-cy, P_contact.x()-cx)
        ref_T_F_C = copy.deepcopy(self.T_F_C)

        print "alpha_init: %s"%(alpha_init)
        print "alpha_door: %s"%(alpha_door)

        # calculate gripper angle
        stop = False
        beta_dest = alpha_door-alpha_init
        beta = 0.0
        time = 0.0
        delta_t = 0.05
        if beta_dest > 0:
            omega = 0.1
        else:
            omega = -0.1
        traj = []
        times = []

        dest_pt = T_B_Wd_init * self.T_W_E * E_pt
        E_pt_in_B = self.T_B_W * self.T_W_E * E_pt
        r_v = dest_pt - E_pt_in_B

        print "beta_dest: %s"%(beta_dest)

        # calculate trajectory for rotating gripper to correct orientation
        while not stop:
            if ((beta_dest > 0) and (beta > beta_dest)) or ((beta_dest <= 0) and (beta < beta_dest)):
                beta = beta_dest
                stop = True
            time += delta_t
            
            dest_pt_d = PyKDL.Frame(PyKDL.Rotation.RotZ(beta)) * r_v + E_pt_in_B
            R_B_Ed = copy.deepcopy((PyKDL.Frame(PyKDL.Rotation.RotZ(beta)) * self.T_B_W * self.T_W_E).M)

            self.publishSinglePointMarker(dest_pt_d.x(), dest_pt_d.y(), dest_pt_d.z(), self.m_id, r=0.0, g=1.0, b=0.0)
            self.m_id += 1

            T_B_Wd = self.calculateMoveGripperPointToPose(E_pt, R_B_Ed, dest_pt_d)

            traj.append(T_B_Wd)
            times.append(time)
#            self.moveWrist2(T_B_Wd)
#            self.checkStopCondition(0.05)
            beta += omega*delta_t

        raw_input("Press Enter to continue...")
        self.checkStopCondition(0.05)
        if self.emergency_stop_active:
            self.failure_reason = "emergency_stop"
            print "end: emergency stop"
            return 0

        # rotate the gripper with spring
        self.moveWristTraj( traj, times, Wrench(Vector3(10,25,25), Vector3(4,3,2)) )
        end_t = rospy.Time.now()+rospy.Duration(time)
        while rospy.Time.now()<end_t:
            self.getTransformations()
            T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
            P_contact = T_B_C*PyKDL.Vector(0,0,0)
            self.addContact(P_contact)

            # for score function
            dur = end_t-rospy.Time.now()
            time_left = dur.to_sec()
            f = time_left / time
            score = f*self.score_open_B + (1.0-f)*self.score_open_C

            if not self.hasContact(50):
                self.emergencyStop()
                self.failure_reason = "lost_contact"
                print "end: lost contact"
                rospy.sleep(1.0)
                return score
            self.checkStopCondition(0.05)
            if self.emergency_stop_active:
                self.failure_reason = "emergency_stop"
                print "end: emergency stop"
                return score

        cx, cy, r = self.estCircle(self.px, self.py)
        r_old = r

        # for score function
        self.init_motion_time_left = 0.0
        self.circle_cx = cx
        self.circle_cy = cy
        self.circle_r = r

        self.publishDoorMarker(cx, cy, pz, r)
        circle = QuaternionStamped()
        circle.header.frame_id = 'torso_base'
        circle.header.stamp = rospy.Time.now()
        circle.quaternion = Quaternion(cx, cy, pz, r)
        self.pub_circle.publish(circle)
        print "circle: x: %s   y: %s    r: %s"%(cx,cy,r)

        #
        # door opening loop
        #
        self.getTransformations()
        # get current contact point
        T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
        P_contact = T_B_C*PyKDL.Vector(0,0,0)
        # calculate current door angle
        ref_alpha_door = math.atan2(P_contact.y()-cy, P_contact.x()-cx)

        ref_T_B_E = self.T_B_W * self.T_W_E

        #ref_dest_pt = traj[-1] * self.T_W_E * self.T_E_F * ref_T_F_C * PyKDL.Vector(0,0,0)
        dest_pt = copy.deepcopy(dest_pt_d)#ref_dest_pt)
        ref_alpha_contact = math.atan2(dest_pt.y()-cy, dest_pt.x()-cx)
        alpha_contact = copy.copy(ref_alpha_contact)
        dest_alpha_door = copy.copy(beta)
        while alpha_door-alpha_init < self.door_angle_dest:
            # get current contact point
            self.getTransformations()
            T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
            P_contact = T_B_C*PyKDL.Vector(0,0,0)
            self.addContact(P_contact)
            # calculate current door angle
            alpha_door = math.atan2(P_contact.y()-cy, P_contact.x()-cx)
            # calculate aplha_contact for old destination point and new estimated circle
            prev_alpha_contact = math.atan2(dest_pt.y()-cy, dest_pt.x()-cx)
            # calculate radius for old destination point and new estimated circle
            current_radius = math.sqrt( (P_contact.x()-cx)*(P_contact.x()-cx) + (P_contact.y()-cy)*(P_contact.y()-cy) )

            alpha_contact = copy.copy(prev_alpha_contact)
            alpha_contact += 0.005
            dest_pt = PyKDL.Vector(cx, cy, pz) + (current_radius + self.r_a)*PyKDL.Vector(math.cos(alpha_contact), math.sin(alpha_contact), 0)

            print "current_radius: %s     alpha_contact: %s"%(current_radius, alpha_contact)

            self.publishSinglePointMarker(dest_pt.x(), dest_pt.y(), dest_pt.z(), self.m_id, r=0.0, g=1.0, b=0.0)
            self.m_id += 1
            beta += 0.01
            if beta > alpha_door-ref_alpha_door:
                beta = alpha_door-ref_alpha_door
            T_B_Wd = self.calculateMoveGripperPointToPose(E_pt, (PyKDL.Frame(PyKDL.Rotation.RotZ(beta)) * ref_T_B_E ).M, dest_pt)
            self.moveWrist2(T_B_Wd)
#            raw_input("Press Enter to continue...")
#            self.checkStopCondition(0.5)

            self.moveWrist(T_B_Wd, 0.1, Wrench(Vector3(10,25,25), Vector3(4,3,2)))
            end_t = rospy.Time.now()+rospy.Duration(0.1)
            while rospy.Time.now()<end_t:
                self.getTransformations()
                T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
                P_contact = T_B_C*PyKDL.Vector(0,0,0)
                self.addContact(P_contact)

                f = ( (alpha_door-alpha_init) - (ref_alpha_door-alpha_init) ) / (self.door_angle_dest - (ref_alpha_door-alpha_init))
                if f > 1.0: f = 1.0
                if f < 0.0: f = 0.0
                score = (1.0-f)*self.score_open_C + f*self.score_open_D

                self.checkStopCondition(0.01)
                if self.emergency_stop_active:
                    self.failure_reason = "emergency_stop"
                    print "end: emergency stop"
                    return score

                if not self.hasContact(50):
                    self.emergencyStop()
                    self.failure_reason = "lost_contact"
                    print "end: lost contact"
                    rospy.sleep(1.0)
                    return score

            cx, cy, r = self.estCircle(self.px, self.py)
            r_old = r
            if r > 0.45 :
                self.failure_reason = "estimation_error_2"
                print "estimation error: too big radius"
                return score
            if r < 0.075 :
                self.failure_reason = "estimation_error_3"
                print "estimation error: too small radius"
                return score

            # for score function
            self.init_motion_time_left = 0.0
            self.circle_cx = cx
            self.circle_cy = cy
            self.circle_r = r

            self.publishDoorMarker(cx, cy, pz, r)
            circle = QuaternionStamped()
            circle.header.frame_id = 'torso_base'
            circle.header.stamp = rospy.Time.now()
            circle.quaternion = Quaternion(cx, cy, pz, r)
            self.pub_circle.publish(circle)
            print "circle: x: %s   y: %s    r: %s"%(cx,cy,r)

        return self.score_open_D

    def updateMarkerPose(self, T_B_M, P_door_surface):
        v1 = PyKDL.Vector( P_door_surface[0][0], P_door_surface[0][1], P_door_surface[0][2] )
        v2 = PyKDL.Vector( P_door_surface[1][0], P_door_surface[1][1], P_door_surface[1][2] )
        v3 = PyKDL.Vector( P_door_surface[2][0], P_door_surface[2][1], P_door_surface[2][2] )

        nz = (v1-v2) * (v2-v3)
        nz.Normalize()

        mz = T_B_M.M * PyKDL.Vector(0, 0, 1)
        if PyKDL.dot(nz, mz) < 0:
            nz = -nz

        mx = T_B_M.M * PyKDL.Vector(1, 0, 0)

        ny = nz * mx
        ny.Normalize()
        nx = ny * nz
        nx.Normalize()

        rot = PyKDL.Rotation(nx, ny, nz)

        dist_m = PyKDL.dot( T_B_M.p, nz )
        dist_n = PyKDL.dot( PyKDL.Vector( P_door_surface[0][0], P_door_surface[0][1], P_door_surface[0][2] ), nz )

        m_p = nz*(dist_n-dist_m) + T_B_M.p
        frame = PyKDL.Frame( rot, m_p )
        return frame

    def handleEmergencyStop(self):
        if self.emergency_stop_active:
            ch = '_'
            while (ch != 'e') and (ch != 'n') and (ch != 'r'):
                ch = raw_input("Emergency stop active... (e)xit, (n)ext case, (r)epeat case: ")
            if ch == 'e':
                exit(0)
            if ch == 'n':
                self.index += 1
            self.getTransformations()
            print "moving desired pose to current pose"
            self.emergency_stop_active = False
            self.moveWrist(self.T_B_W, 2.0, Wrench(Vector3(20,20,15), Vector3(6,4,2)))
            self.checkStopCondition(2.0)
            return True
        return False

    def printQualityMeasure(self, score_open):
        print "quality measure:"

        print "init_motion_time_left: %s"%(self.init_motion_time_left)
        print "circle_cx: %s"%(self.circle_cx)
        print "circle_cy: %s"%(self.circle_cy)
        print "circle_r: %s"%(self.circle_r)

        wrench_total = math.sqrt(self.wrench_max.force.x*self.wrench_max.force.x + self.wrench_max.force.y*self.wrench_max.force.y + self.wrench_max.force.z*self.wrench_max.force.z) + 10*math.sqrt(self.wrench_max.torque.x*self.wrench_max.torque.x + self.wrench_max.torque.y*self.wrench_max.torque.y + self.wrench_max.torque.z*self.wrench_max.torque.z)
        wrench_mean_total = math.sqrt(self.wrench_mean.force.x*self.wrench_mean.force.x + self.wrench_mean.force.y*self.wrench_mean.force.y + self.wrench_mean.force.z*self.wrench_mean.force.z) + 10*math.sqrt(self.wrench_mean.torque.x*self.wrench_mean.torque.x + self.wrench_mean.torque.y*self.wrench_mean.torque.y + self.wrench_mean.torque.z*self.wrench_mean.torque.z)

        print "total max wrench: %s"%(wrench_total)
        print "total mean wrench: %s"%(wrench_mean_total)
        print "score_open: %s"%(score_open)
        print "total score: %s"%(wrench_total+wrench_mean_total+score_open)
        print "failure_reason: %s"%(self.failure_reason)

        with open("experiments.txt", "a") as exfile:
            exfile.write( "quality measure:" + "\n")
            exfile.write( "init_motion_time_left:" + str(self.init_motion_time_left) + "\n" )
            exfile.write( "circle_cx:" + str(self.circle_cx) + "\n" )
            exfile.write( "circle_cy:" + str(self.circle_cy) + "\n" )
            exfile.write( "circle_r:" + str(self.circle_r) + "\n" )
            exfile.write( "total max wrench:" + str(wrench_total) + "\n" )
            exfile.write( "total mean wrench:" + str(wrench_mean_total) + "\n" )
            exfile.write( "score_open:" + str(score_open) + "\n" )
            exfile.write( "total score:" + str(wrench_total+wrench_mean_total+score_open) + "\n" )
            exfile.write( "failure_reason:" + self.failure_reason + "\n" )

    def updateTool(self):
        print "setting the tool to %s relative to wrist frame"%(self.T_W_T)
        # move both tool position and wrist position - the gripper holds its position
        print "moving wrist"
        # we assume that during the initialization there are no contact forces, so we limit the wrench
        self.moveWrist( self.T_B_W, 2.0, Wrench(Vector3(5, 5, 5), Vector3(2, 2, 2)) )
        print "moving tool"
        self.moveTool( self.T_W_T, 2.0 )
        self.checkStopCondition(2.0)

    def spin(self, door):

        # start with very low stiffness
        print "setting stiffness to very low value"
        self.moveImpedance(self.k_error, 0.5)
        self.checkStopCondition(0.5)

        raw_input("Press Enter to continue...")
        self.checkStopCondition()

        self.getTransformations()

        self.T_W_T = PyKDL.Frame(PyKDL.Vector(0.2,-0.05,0))    # tool transformation
        self.updateTool()

        # change the stiffness
        print "changing stiffness for door approach"
        self.moveImpedance(self.k_door, 2.0)
        self.checkStopCondition(2.0)

        # straighten fingers
        self.move_hand_client(self.prefix, self.q_start)

        rospy.sleep(2.0)

        # uncomment the following line to get marker pose from vision
        door_marker = None
#        door_marker = ((0.8706762604472923, -0.17540538568541159, 1.2997999734039194), (-0.46361347288013666, 0.5215836270730932, 0.5478510309867619, -0.4613808790793538))

        if door_marker == None:
            if self.door_marker_visible:
                print "Found door marker"
            else:
                self.emergencyStop()
                print "Could not find door marker"
                rospy.sleep(1.0)
                return

            self.checkStopCondition()

            # get door marker absolute position
            self.listener.waitForTransform('torso_base', 'ar_marker_3', rospy.Time.now(), rospy.Duration(4.0))
            door_marker = self.listener.lookupTransform('torso_base', 'ar_marker_3', rospy.Time(0))

        print "door marker:"
        print door_marker

        door.getAttribute("base").value = pm.fromTf(door_marker)

        find_door_surface = True

        if find_door_surface:
            # approach the door
            self.move_hand_client(self.prefix, self.q_door)
            rospy.sleep(2.0)

            P_s = []
            P_s.append(door.getAttribute("P1_start").value)
            P_s.append(door.getAttribute("P2_start").value)
            P_s.append(door.getAttribute("P3_start").value)
            P_e = []
            P_e.append(door.getAttribute("P1_end").value)
            P_e.append(door.getAttribute("P2_end").value)
            P_e.append(door.getAttribute("P3_end").value)

            P_door_surface = []
            self.getTransformations()
            for i in range(0, 3):
                T_B_Nd = door.getAttribute("base").value * P_s[i]
                T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
                self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(15,15,15), Vector3(3,3,3)))
                self.checkStopCondition(3.0)

                T_B_Nd = door.getAttribute("base").value * P_e[i]
                T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
                self.moveWrist(T_B_Wd, 8.0, Wrench(Vector3(15,15,15), Vector3(3,3,3)))

                contact_found = False
                end_t = rospy.Time.now()+rospy.Duration(8.0)
                while rospy.Time.now()<end_t:
                    self.checkStopCondition(0.05)
                    if self.hasContact(100):
                        contact_found = True
                        self.stopArm()
                        print "found contact"
                        break
                if not contact_found:
                    self.emergencyStop()
                    rospy.sleep(1.0)
                    return

                self.getTransformations()
                T_B_C = self.T_B_W * self.T_W_E * self.T_E_F * self.T_F_C
                P_door_surface.append( T_B_C * PyKDL.Vector(0, 0, 0) )

                T_B_Nd = door.getAttribute("base").value * P_s[i]
                T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
                self.moveWrist(T_B_Wd, 2.0, Wrench(Vector3(15,15,15), Vector3(3,3,3)))
                self.checkStopCondition(2.0)

            print P_door_surface

            print "before:"
            print door.getAttribute("base").value
            door.getAttribute("base").value = self.updateMarkerPose(door.getAttribute("base").value, P_door_surface)
            print "after:"
            print door.getAttribute("base").value

            # straighten fingers
            self.move_hand_client(self.prefix, self.q_start)
            self.checkStopCondition(2.5)

        self.moveImpedance(self.k_door2, 2.0)
        self.move_hand_client(self.prefix, self.q_handle)
        self.checkStopCondition(2.0)

        self.getTransformations()

        # set tool at middle finger nail, with the same orientation as wrist
        self.T_W_T = PyKDL.Frame((self.T_W_E * self.T_E_F * self.T_F_N)*PyKDL.Vector(0,0,0))    # tool transformation
        self.updateTool()

        self.getTransformations()

        raw_input("Press Enter to continue...")

        # the orientation of E is set according to orientation of M (door.base)
        # the finger nail frame N is at position door.H_start

        # calculate rotation of gripper in door base frame M
        T_M_Ed = PyKDL.Frame( PyKDL.Rotation.RotZ(-math.pi/2.0) * PyKDL.Rotation.RotX(math.pi) )
        # calculate rotation of gripper in robot base frame B
        R_B_Ed = copy.deepcopy((door.getAttribute("base").value * T_M_Ed).M)

        T_B_Wd = self.calculateMoveGripperPointToPose( (self.T_E_F * self.T_F_N) * PyKDL.Vector(0,0,0), R_B_Ed, door.getAttribute("base").value * door.getAttribute("H_start").value )
        self.moveWrist(T_B_Wd, 4.0, Wrench(Vector3(15,15,15), Vector3(3,3,3)))
        self.checkStopCondition(4.0)

#######################################
        if False:
            raw_input("Press Enter to continue...")

            start_stamp = rospy.Time.now() + rospy.Duration(1.0)
            self.moveImpedance(Wrench(Vector3(10.0, 800.0, 800.0), Vector3(300.0, 300.0, 300.0)), 0.5, stamp=start_stamp)
            self.action_impedance_client.wait_for_result()
            print "result: %s"%(self.action_impedance_client.get_result())
            print "status: %s"%(self.action_impedance_client.get_state())

            return
            raw_input("Press Enter to continue...")

            start_stamp = rospy.Time.now() + rospy.Duration(1.0)
            print "result: %s"%(self.action_impedance_client.get_result())
            print "status: %s"%(self.action_impedance_client.get_state())

            raw_input("Press Enter to continue...")

            self.checkStopCondition(1.0)


#######################################

        raw_input("Press Enter to continue...")

        # the orientation of E is set according to orientation of M (door.base)
        # the finger nail frame N is at position door.H_end
        T_B_Wd = self.calculateMoveGripperPointToPose( (self.T_E_F * self.T_F_N) * PyKDL.Vector(0,0,0), R_B_Ed, door.getAttribute("base").value * door.getAttribute("H_end").value )
        self.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(15,15,15), Vector3(3,3,3)))

        contact_found = False
        end_t = rospy.Time.now()+rospy.Duration(8.0)
        while rospy.Time.now()<end_t:
            self.checkStopCondition(0.05)
            if self.hasContact(100):
                contact_found = True
                self.stopArm()
                print "found contact"
                break
        if not contact_found:
            self.emergencyStop()
            rospy.sleep(1.0)
            return

        self.getTransformations()
        T_B_C = self.T_B_W * self.T_W_E * self.T_E_F * self.T_F_C
        T_M_B = door.getAttribute("base").value.Inverse()
        door.getAttribute("handle").value = T_M_B * T_B_C
        door.getAttribute("pre_handle").value = T_M_B * T_B_C

        print door.getAttribute("handle").value
        door.getAttribute("handle").value.p += PyKDL.Vector(-0.0, 0.0, -0.03)
        door.getAttribute("pre_handle").value.p += PyKDL.Vector(0.10, 0.0, 0.03)

        print door.getAttribute("handle").value
        print door.getAttribute("pre_handle").value

        raw_input("Press Enter to continue...")

        self.exit_on_emergency_stop = False

        forces = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5,]
        k_x = [500.0]
        k_y = [10.0, 20.0, 40.0, 80.0]

        with open("experiments.txt", "a") as exfile:
            exfile.write("******** experiment series begin ********" + "\n")


        self.max_index = len(forces) * len(k_x) * len(k_y)

        self.index = 0
        # door opening loop
        while self.index < self.max_index:
            self.failure_reason = "unknown"

            i = 0
            for forces_v in forces:
                for k_x_v in k_x:
                    for k_y_v in k_y:
                        if i == self.index:
                            self.force = forces_v
                            self.learning_k_handle_x = k_x_v
                            self.learning_k_open_x = k_x_v
                            self.learning_k_open_y = k_y_v
                            self.learning_r_a = self.force / self.learning_k_open_y
                        i += 1

            self.r_a = self.learning_r_a
            self.k_open = Wrench(Vector3(self.learning_k_open_x, self.learning_k_open_y, 800.0), Vector3(300.0, 300.0, 300.0))

            print "parameters:"
            print "index: %s"%(self.index)
            print "force: %s"%(self.force)
            print "r_a: %s"%(self.learning_r_a)
            print "k_handle_x: %s"%(self.learning_k_handle_x)
            print "k_open_y: %s"%(self.learning_k_open_y)

            with open("experiments.txt", "a") as exfile:
                exfile.write("parameters:\n")
                exfile.write("index:"+str(self.index) + "\n")
                exfile.write("force:"+str(self.force) + "\n")
                exfile.write("r_a:"+str(self.learning_r_a) + "\n")
                exfile.write("k_handle_x:"+str(self.learning_k_handle_x) + "\n")
                exfile.write("k_open_y:"+str(self.learning_k_open_y) + "\n")

            self.moveImpedance(self.k_door2, 2.0)
            self.checkStopCondition(2.0)

            self.getTransformations()

            T_B_Nd = door.getAttribute("base").value * door.getAttribute("pre_handle").value
            T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
            self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(20,20,20), Vector3(3,3,3)))
            self.checkStopCondition(3.0)

            if self.handleEmergencyStop():
                continue

            raw_input("Press Enter to continue...")

            T_B_Nd = door.getAttribute("base").value * door.getAttribute("handle").value
            T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
            self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(20,20,20), Vector3(3,3,3)))
            self.checkStopCondition(3.0)

            if self.handleEmergencyStop():
                continue

#            raw_input("Press Enter to continue...")

            print "pushing the handle"

            # calculate the force
            F_y = self.k_open.force.y * self.r_a
            # calculate initial r_a
            r_a_interpolated = F_y / self.k_door2.force.y
            k_interpolated = copy.deepcopy(self.k_door2)
            print "calculated: F_y=%s    r_a_interpolated=%s"%(F_y, r_a_interpolated)

            raw_input("Press Enter to continue...")

            print "pushing handle with current stiffness"
            T_B_Nd = door.getAttribute("base").value * PyKDL.Frame(PyKDL.Vector(-r_a_interpolated, 0, 0)) * door.getAttribute("handle").value
            T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
            self.moveWrist(T_B_Wd, 2.0, Wrench(Vector3(20,20,20), Vector3(3,3,3)))
            self.checkStopCondition(2.0)

            velocity = 0.05    # 5 cm/s
            delta_t = 0.01    # 10 ms
            traj = []
            traj_imp = []
            times = []
            time = 0.0
            stop = False
            steps = int((self.r_a - r_a_interpolated) / (velocity * delta_t))
            print steps
            step = 0
            while not stop:
                r_a_interpolated += velocity * delta_t
                time += delta_t
                if r_a_interpolated > self.r_a:
                    r_a_interpolated = self.r_a
                    k_interpolated.force.x = self.k_open.force.x
                    stop = True
                else:
                    f = float(step)/steps
                    k_interpolated.force.x = (1.0-f)*self.k_door2.force.x + f*self.k_open.force.x
                k_interpolated.force.y = F_y/r_a_interpolated
                T_B_Nd = door.getAttribute("base").value * PyKDL.Frame(PyKDL.Vector(-r_a_interpolated, 0, 0)) * door.getAttribute("handle").value
                T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
                traj.append(T_B_Wd)
                traj_imp.append(k_interpolated)
                times.append(time)
#                print "%s:  %s    %s   %s"%(step, time, T_B_Wd.p, k_interpolated.force)
                step += 1

#            raw_input("Press Enter to continue...")
#            self.checkStopCondition(0.5)

            if self.handleEmergencyStop():
                self.printQualityMeasure(1000)
                continue

            start_stamp = rospy.Time.now() + rospy.Duration(1.0)
            self.moveImpedanceTraj( traj_imp, times, stamp=start_stamp)
            self.moveWristTraj(traj, times, Wrench(Vector3(20,20,20), Vector3(3,3,3)), stamp=start_stamp)
            self.checkStopCondition(time+1.5)

            if self.handleEmergencyStop():
                self.printQualityMeasure(1000)
                continue

            score_open = self.openTheDoor(R_B_Ed)

            self.printQualityMeasure(score_open)

            if self.handleEmergencyStop():
                continue

            raw_input("Press Enter to stop pulling the handle...")
            self.stepBackFromHandle()

            if self.handleEmergencyStop():
                continue
            raw_input("Press Enter to continue...")

            self.index += 1

        return

if __name__ == '__main__':

    door = Door()

    rot = PyKDL.Rotation.RotZ(-math.pi/2.0) * PyKDL.Rotation.RotY(math.pi/2.0) * PyKDL.Rotation.RotZ(-10.0*math.pi/180.0)
    door.getAttribute("P1_start").value = PyKDL.Frame( rot, PyKDL.Vector(0.10, -0.12, 0.10) )
    door.getAttribute("P1_end").value   = PyKDL.Frame( rot, PyKDL.Vector(0.10, -0.12, -0.10) )
    door.getAttribute("P2_start").value = PyKDL.Frame( rot, PyKDL.Vector(0.0, 0.0, 0.10) )
    door.getAttribute("P2_end").value   = PyKDL.Frame( rot, PyKDL.Vector(0.0, 0.0, -0.10) )
    door.getAttribute("P3_start").value = PyKDL.Frame( rot, PyKDL.Vector(0.10, 0, 0.10) )
    door.getAttribute("P3_end").value   = PyKDL.Frame( rot, PyKDL.Vector(0.10, 0, -0.10) )

    # for small radius
#    door.getAttribute("H_start").value = PyKDL.Vector(0.10, -0.12, 0.005)
#    door.getAttribute("H_end").value   = PyKDL.Vector(-0.10, -0.12, 0.005)

    # for big radius
    door.getAttribute("H_start").value = PyKDL.Vector(0.0, -0.12, 0.005)
    door.getAttribute("H_end").value   = PyKDL.Vector(-0.20, -0.12, 0.005)

    # for test
#    door.getAttribute("H_start").value = PyKDL.Vector(0.0, -0.12, 0.10)
#    door.getAttribute("H_end").value   = PyKDL.Vector(-0.20, -0.12, 0.10)

    door.printAttributes()

    rospy.init_node('door_opener')
    doorOpener = DoorOpener()

    try:
        doorOpener.spin(door)
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass

