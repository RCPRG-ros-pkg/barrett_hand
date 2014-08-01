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
import scipy.io as sio

import PyKDL
import math
from numpy import *
import numpy as np
from scipy import optimize

import copy

from scipy.interpolate import *
import matplotlib.pyplot as plt

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

class VectorAttribute(Attribute):
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
        self.attributes.append( VectorAttribute("H_start", None) )      # handle search
        self.attributes.append( VectorAttribute("H_end", None) )        # handle search
        self.attributes.append( ScalarAttribute("handle_radius", None) )
        self.attributes.append( VectorAttribute("handle", None) )
        self.attributes.append( VectorAttribute("pre_handle", None) )
        self.attributes.append( VectorAttribute("hinge_pos", None) )

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
        max_tactile_value = -1.0
        max_tactile_index = 0
        max_tactile_finger = 0
        fingers = [data.finger1_tip, data.finger2_tip, data.finger3_tip]
        contact_center = PyKDL.Vector()
        val_sum = 0.0
        for f in range(0,3):
            for i in range(0, 24):
                if fingers[f][i] > max_tactile_value:
                    max_tactile_value = fingers[f][i]
                    max_tactile_index = i
                    max_tactile_finger = f
                if fingers[f][i] > 50:
                    val = fingers[f][i]# - 50.0
                    pt = self.pressure_info.sensor[f].center[i]
                    contact_center += PyKDL.Vector(pt.x, pt.y, pt.z)*val
                    val_sum += val
        self.tactile_lock.acquire()
        if val_sum > 0:
            self.tactile_data_index += 1
            if self.tactile_data_index >= self.tactile_data_len:
                self.tactile_data_index = 0

            contact_center *= 1.0/val_sum
            self.max_tactile_value = copy.copy(max_tactile_value)
            # time, finger, max value, contact center
            self.tactile_data[self.tactile_data_index][0] = copy.copy(data.header.stamp)
            self.tactile_data[self.tactile_data_index][1] = copy.copy(max_tactile_finger)
            self.tactile_data[self.tactile_data_index][2] = copy.copy(max_tactile_value)
            self.tactile_data[self.tactile_data_index][3] = copy.deepcopy(contact_center)
        self.tactile_lock.release()


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

        ws = WrenchStamped()
        ws.header.stamp = rospy.Time.now()
        ws.header.frame_id = self.prefix+"_HandPalmLink"
        ws.wrench = wrench
        self.pub_wrench.publish(ws)

    def resetMarkCounters(self):
        self.wrench_max = Wrench()
        self.wrench_mean_count = 0
        self.wrench_mean = Wrench()

    def __init__(self):
        # parameters
        self.prefix="right"
        self.q_start = (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 180.0/180.0*numpy.pi) 
        self.q_door = (120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
        self.q_handle = (75.0/180.0*numpy.pi, 75.0/180.0*numpy.pi, 75.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
        self.q_close = (120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
        self.d_init = 0.1
        self.k_door = Wrench(Vector3(200.0, 800.0, 1000.0), Vector3(300.0, 300.0, 300.0))
        self.k_door2 = Wrench(Vector3(800.0, 800.0, 800.0), Vector3(300.0, 300.0, 300.0))
        self.k_error = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))
        self.k_close = Wrench(Vector3(400.0, 400.0, 400.0), Vector3(100.0, 100.0, 100.0))
        self.T_B_W = None
        self.T_W_T = PyKDL.Frame(PyKDL.Vector(0.2,-0.05,0))    # tool transformation
        self.T_W_E = None
        self.T_E_W = None
        self.T_F_N = PyKDL.Frame( PyKDL.Vector(0.03, -0.01, 0) )
        self.T_N_F = self.T_F_N.Inverse()
        self.current_max_wrench = Wrench(Vector3(20, 20, 20), Vector3(20, 20, 20))
        self.wrench_emergency_stop = False
        self.exit_on_emergency_stop = True

        self.last_contact_time = rospy.Time.now()

        self.init_motion_time = 3.0
        self.door_angle_dest = 80.0/180.0*math.pi

        self.tactile_lock = Lock()

        # for tactile sync
        self.tactile_data = []
        self.tactile_data_len = 40
        self.tactile_data_index = 0
        for i in range(0, self.tactile_data_len):
            # time, finger, max value, contact center
            self.tactile_data.append( [rospy.Time.now(), 0, 0, PyKDL.Vector()] )

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

        self.pub_wrench = rospy.Publisher("/"+self.prefix+"_arm/wrench_stamped", WrenchStamped)

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

        print "Requesting pressure sensors info"
        self.pressure_info = self.get_pressure_sensors_info_client()
        self.pressure_frames = []
        for i in range(0, 24):
            center = PyKDL.Vector(self.pressure_info.sensor[0].center[i].x, self.pressure_info.sensor[0].center[i].y, self.pressure_info.sensor[0].center[i].z)
            halfside1 = PyKDL.Vector(self.pressure_info.sensor[0].halfside1[i].x, self.pressure_info.sensor[0].halfside1[i].y, self.pressure_info.sensor[0].halfside1[i].z)
            halfside2 = PyKDL.Vector(self.pressure_info.sensor[0].halfside2[i].x, self.pressure_info.sensor[0].halfside2[i].y, self.pressure_info.sensor[0].halfside2[i].z)
            halfside1.Normalize()
            halfside2.Normalize()
            norm = halfside1*halfside2
            norm.Normalize()
            self.pressure_frames.append( PyKDL.Frame(PyKDL.Rotation(halfside1, halfside2, norm), center) )

        # calculate desired point of contact (middle of tactile sensor) in E frame
        self.T_F_pt = self.pressure_frames[7]

        self.wrench_tab = []
        self.wrench_tab_index = 0
        self.wrench_tab_len = 4000
        for i in range(0,self.wrench_tab_len):
            self.wrench_tab.append( Wrench(Vector3(), Vector3()) )

        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvarMarkerCallback)
        rospy.Subscriber('/'+self.prefix+'_hand/BHPressureState', BHPressureState, self.tactileCallback)
        rospy.Subscriber('/'+self.prefix+'_arm/wrench', Wrench, self.wrenchCallback)

    def moveWrist2(self, wrist_frame):
        wrist_pose = pm.toMsg(wrist_frame)
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

    def estLine(self, px, py):
        n = len(px)
        if n<2:
            return [1,0,0]
        def calc_dist(angle):
            a = cos(angle)
            b = sin(angle)
            c = -math.fsum(a*numpy.asarray(px) + b*numpy.asarray(py))/n
            return a*numpy.asarray(px) + b*numpy.asarray(py) + c

        def f_2(p):
            Ri = calc_dist(*p)
            return Ri - Ri.mean()

        line_estimate = 0.0
        line_2, ier = optimize.leastsq(f_2, line_estimate, maxfev = 2000)

        angle = line_2
        a = cos(angle)
        b = sin(angle)
        c = -math.fsum(a*numpy.asarray(px) + b*numpy.asarray(py))/n
        return [a, b, c]

    def estCircle(self, px, py, x_m=None, y_m=None):
      if x_m == None:
         x_m = mean(px)
      if y_m == None:
         y_m = mean(py)

      def calc_R(xc, yc):
        """ calculate the distance of each 2D points from the center (xc, yc) """
        return sqrt((px-xc)**2 + (py-yc)**2)

      def f_2(c):
        """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
        Ri = calc_R(*c)
        return Ri - Ri.mean()

      center_estimate = x_m, y_m
      center_2, ier = optimize.leastsq(f_2, center_estimate, maxfev = 2000)

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

    def publishVectorMarker(self, v1, v2, i, r, g, b, frame):
        m = MarkerArray()

        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'door'
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = 0
        marker.points.append(Point(v1.x(), v1.y(), v1.z()))
        marker.points.append(Point(v2.x(), v2.y(), v2.z()))
        marker.pose = Pose( Point(0,0,0), Quaternion(0,0,0,1) )
        marker.scale = Vector3(0.001, 0.002, 0)
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)

        self.pub_marker.publish(m)

    def clearDoorEstMarkers(self):
        m = MarkerArray()
        for i in range(0, 1000):
            marker = Marker()
            marker.header.frame_id = 'torso_base'
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'door'
            marker.id = i
            marker.type = 0
            marker.action = Marker.DELETE
            m.markers.append(marker)
        self.pub_marker.publish(m)

    def hasContact(self, threshold, print_on_false=False):
        if self.T_F_C != None:
            return True
        return False

    def getTransformations(self):
        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))
        self.T_B_W = pm.fromTf(pose)

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        self.T_E_F = pm.fromTf(pose)
        self.T_F_E = self.T_E_F.Inverse()

        if self.T_W_E == None:
            pose = self.listener.lookupTransform(self.prefix+'_arm_7_link', self.prefix+'_HandPalmLink', rospy.Time(0))
            self.T_W_E = pm.fromTf(pose)
            self.T_E_W = self.T_W_E.Inverse()

    def getTransformationsForContact(self, threshold = 100):
        self.tactile_lock.acquire()
        index = copy.copy(self.tactile_data_index)
        max_value = copy.copy(self.max_tactile_value)
        self.tactile_lock.release()

        self.T_F_C = None
        self.T_C_F = None

        if max_value<threshold:
            return

        for i in range(0, self.tactile_data_len-1):
            # time, finger, max value, contact center
            time = self.tactile_data[index][0]
            if self.listener.canTransform('torso_base', self.prefix+'_arm_7_link', time) and self.listener.canTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', time):
                self.T_F_C = PyKDL.Frame( copy.deepcopy(self.tactile_data[index][3]) )
                self.T_C_F = self.T_F_C.Inverse()
                self.T_B_W = pm.fromTf(self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', time))
                self.T_E_F = pm.fromTf(self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', time))
                self.T_F_E = self.T_E_F.Inverse()
                self.last_contact_time = copy.copy(time)

                break
            index -= 1
            if index < 0:
                index = copy.copy(self.tactile_data_len)-1

    def stepBackFromHandle(self):
        self.getTransformations()
        print "moving desired pose to current pose"
        self.moveWrist(self.T_B_W, 2.0, Wrench(Vector3(25,25,25), Vector3(6,5,4)))
        self.checkStopCondition(2.0)

        self.getTransformations()
        print "moving gripper outwards the handle"
        T_W_Wd = PyKDL.Frame(PyKDL.Vector(0,-0.05,0))
        T_B_Wd = self.T_B_W*T_W_Wd
        self.moveWrist(T_B_Wd, 0.5, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        self.checkStopCondition(0.5)

        self.getTransformations()
        print "rotating gripper outwards the handle"
        T_W_Wd = PyKDL.Frame(PyKDL.Rotation.RotZ(-10.0/180.0*math.pi))
        T_B_Wd = self.T_B_W*T_W_Wd
        self.moveWrist(T_B_Wd, 2.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
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
        T_B_Ed = PyKDL.Frame( copy.deepcopy(rot_B_E), pos_B_E_p - P_B_N )
        T_B_Wd = T_B_Ed * self.T_E_W
        return T_B_Wd

########################
## door opening start ##
########################

    def fixAngleDiff(self, angle_diff):
        if angle_diff > math.pi:
            return angle_diff - math.pi*2.0
        if angle_diff < -math.pi:
            return angle_diff + math.pi*2.0
        return angle_diff

    def resetArc(self):
        self.px = []
        self.py = []
        self.pz = 0.0
        self.arc_state = 0

    def calculateArc(self):
        TR_B_M = copy.deepcopy(PyKDL.Frame(door.getAttribute("base").value.M))
        n = TR_B_M*PyKDL.Vector(1,0,0)

        # perpendicular door surface
        if self.arc_state == 0:
            if len(self.px) < 1:
                return [n.x(), n.y(), 0.0, 0.0]
            # find contact points' minimum and maximum distance from the door
            min_dist = 10000.0
            max_dist = -10000.0
            T_M_B = door.getAttribute("base").value.Inverse()
            for i in range(0, len(self.px)):
                P_contact_M = T_M_B * PyKDL.Vector(self.px[i], self.py[i], self.pz)
                if P_contact_M.z() > max_dist:
                    max_dist = P_contact_M.z()
                if P_contact_M.z() < min_dist:
                    min_dist = P_contact_M.z()
            if max_dist-min_dist > 0.02:
                self.arc_state = 1
            else:
                a = n.x()
                b = n.y()
                c = -math.fsum(a*numpy.asarray(self.px) + b*numpy.asarray(self.py))/len(self.px)
                return [a, b, c, max_dist-min_dist]

        # line estimated from contact points
        if self.arc_state == 1:
            line = self.estLine(self.px, self.py)
            t = PyKDL.Vector(line[1], -line[0], 0)
            min_dist = 10000.0
            max_dist = -10000.0
            for i in range(0, len(self.px)):
                dist = PyKDL.dot( PyKDL.Vector(self.px[i], self.py[i], self.pz), t )
                if dist > max_dist:
                    max_dist = dist
                if dist < min_dist:
                    min_dist = dist
            if max_dist-min_dist > 0.1:
                self.arc_state = 2
            else:
                TR_B_M = copy.deepcopy(PyKDL.Frame(door.getAttribute("base").value.M))
                v = TR_B_M*PyKDL.Vector(1,0,0)
                if PyKDL.dot(v, PyKDL.Vector(line[0], line[1], 0)) < 0:
                    v = TR_B_M*PyKDL.Vector(-1,0,0)
                angle_line = math.atan2(line[1], line[0])
                angle_door = math.atan2(v.y(), v.x())
                angle_line = angle_door + 2.0*self.fixAngleDiff(angle_line - angle_door)
                a = math.cos(angle_line)
                b = math.sin(angle_line)
                c = -a*self.px[-1] - b*self.py[-1]
                if PyKDL.dot(n, PyKDL.Vector(a,b,0)) < 0.0:
                    return [-a, -b, -c, max_dist-min_dist]
                else:
                    return [a, b, c, max_dist-min_dist]

        # circle estimated from contact points
        if self.arc_state == 2:
            cx, cy, r = self.estCircle(self.px,self.py)
            min_angle = math.atan2(self.py[0]-cy, self.px[0]-cx)
            max_angle = math.atan2(self.py[-1]-cy, self.px[-1]-cx)
            diff = self.fixAngleDiff(max_angle-min_angle)
            a = self.px[-1]-cx
            b = self.py[-1]-cy
            c = -a*self.px[-1] - b*self.py[-1]
            if PyKDL.dot(n, PyKDL.Vector(a,b,0)) < 0.0:
                return [-a, -b, -c, diff*r]
            else:
                return [a, b, c, diff*r]

    def openTheDoor(self):
        self.resetArc()
        m_id_start = 10
        m_id_max = 200
        self.m_id = m_id_start

        self.resetMarkCounters()

        self.getTransformations()

        self.E_pt = self.T_E_F * self.T_F_pt * PyKDL.Vector(0,0,0)

        # calculate rotation of gripper in door base frame M
        T_M_Ed = PyKDL.Frame( PyKDL.Rotation.RotZ(-math.pi/2.0) * PyKDL.Rotation.RotX(math.pi) )
        # calculate rotation of gripper in robot base frame B
        R_B_Ed = copy.deepcopy((door.getAttribute("base").value * T_M_Ed).M)

        T_B_Wd = self.calculateMoveGripperPointToPose( self.E_pt, R_B_Ed, door.getAttribute("base").value * door.getAttribute("pre_handle").value )
        self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        self.checkStopCondition(3.1)

        self.getTransformations()

        # radial force
        Fr_value = 8.0
        # initial tangent force
        Ft_value = 2.0

        Fr_versor_B = PyKDL.Frame(door.getAttribute("base").value.M) * PyKDL.Vector(-1.0, 0.0, 0.0)
        Ft_versor_B = PyKDL.Frame(door.getAttribute("base").value.M) * PyKDL.Vector(0.0, 0.0, 1.0)

        v_n = (self.T_B_W * self.T_W_E * self.T_E_F * self.T_F_pt * PyKDL.Vector(0,0,1)) - (self.T_B_W * self.T_W_E * self.T_E_F * self.T_F_pt * PyKDL.Vector(0,0,0))
        alpha_norm = math.atan2(v_n.y(), v_n.x())

        # we assume that initial radius is parallel to the door surface
        F_B = Fr_versor_B * Fr_value + Ft_versor_B * Ft_value
        alpha_force = math.atan2(F_B.y(), F_B.x())

        print "alpha_norm: %s    alpha_force: %s"%(alpha_norm, alpha_force)
        alpha = self.fixAngleDiff(alpha_force-alpha_norm)

        # generate trajectory for rotation of gripper in door base frame M
        T_M_Ed = PyKDL.Frame( PyKDL.Rotation.RotZ(-math.pi/2.0) * PyKDL.Rotation.RotX(math.pi) )

        traj = []
        times = []
        alpha_prim = 0
        omega = 0.2
        if alpha < 0:
            omega = -0.2
        time = 0
        time_d = 0.1
        stop = False
        while not stop:
            if ((alpha > 0) and (alpha_prim > alpha)) or ((alpha <= 0) and (alpha_prim < alpha)):
                alpha_prim = alpha
                stop = True
            time += time_d
            
            # calculate rotation of gripper in robot base frame B
            T_E_Ed = PyKDL.Frame(PyKDL.Rotation.RotX(-alpha_prim))
            R_B_Ed = copy.deepcopy((door.getAttribute("base").value * T_M_Ed * T_E_Ed).M)
            T_B_Wd = self.calculateMoveGripperPointToPose( self.E_pt, R_B_Ed, door.getAttribute("base").value * door.getAttribute("pre_handle").value )

            traj.append(T_B_Wd)
            times.append(time)
            alpha_prim += omega*time_d

        raw_input("Press Enter to continue...")
        if self.checkStopCondition(0.01):
            return 0

        self.moveWristTraj(traj, times, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        self.checkStopCondition(time+0.1)

        # move to "handle" position
        T_E_Ed = PyKDL.Frame(PyKDL.Rotation.RotX(-alpha))
        R_B_Ed = copy.deepcopy((door.getAttribute("base").value * T_M_Ed * T_E_Ed).M)
        T_B_Wd = self.calculateMoveGripperPointToPose( self.E_pt, R_B_Ed, door.getAttribute("base").value * door.getAttribute("handle").value )
        self.moveWrist2(T_B_Wd)

        raw_input("Press Enter to continue...")
        if self.checkStopCondition(0.01):
            return 0

        self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(25,25,25), Vector3(5,5,5)))
        self.checkStopCondition(3.1)

        # change the stiffness
        self.k_open = copy.deepcopy( Wrench(Vector3(800.0, 800.0, 200.0), Vector3(200.0, 200.0, 200.0)) )
        self.moveImpedance(self.k_open, 2.0)
        self.checkStopCondition(2.0)

        # push the gripper into the handle to get in contact
        self.getTransformations()
        T_E_Ed = PyKDL.Frame(PyKDL.Rotation.RotX(-alpha))
        R_B_Ed = copy.deepcopy((door.getAttribute("base").value * T_M_Ed * T_E_Ed).M)
        T_B_Wd = self.calculateMoveGripperPointToPose( self.E_pt, R_B_Ed, door.getAttribute("base").value * door.getAttribute("handle").value + PyKDL.Frame(copy.deepcopy((self.T_B_W * self.T_W_T).M)) * PyKDL.Vector(0, 0, 0.1) )
        self.moveWrist2(T_B_Wd)

        raw_input("Press Enter to continue...")
        if self.checkStopCondition(0.01):
            return 0

        self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(25,25,25), Vector3(5,5,5)))

        contact_found = False
        end_t = rospy.Time.now()+rospy.Duration(3.0)
        while rospy.Time.now()<end_t:
            self.checkStopCondition(0.05)
            self.getTransformationsForContact(100)
            if self.hasContact(100):
                contact_found = True
                self.stopArm()
                print "found contact"
                break
        if not contact_found:
            self.emergencyStop()
            rospy.sleep(1.0)
            return

        no_contact = 0
        hinge = door.getAttribute("base").value*door.getAttribute("hinge_pos").value
        self.pz = (self.T_B_W * self.T_W_E * self.T_E_F * self.T_F_pt * PyKDL.Vector(0,0,0) ).z()

        init_surf = PyKDL.Frame(copy.deepcopy(door.getAttribute("base").value.M)) * PyKDL.Vector(-1,0,0)
        init_angle = math.atan2(init_surf.y(), init_surf.x()) 

        state = 0
        door_position = 0.01
        while True:

            self.getTransformationsForContact(100)            
            contact_time_diff = self.last_contact_time-rospy.Time.now()

            if self.hasContact(100) and contact_time_diff.to_sec() < 0.2:
                T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
                P_contact = T_B_C*PyKDL.Vector(0,0,0)
                self.addContact(P_contact)

            if len(self.px)>1:
                cx, cy, r = self.estCircle(self.px, self.py, hinge.x(), hinge.y())
                r_old = r
                # for score function
                self.circle_cx = cx
                self.circle_cy = cy
                self.circle_r = r
                self.publishDoorMarker(cx, cy, P_contact.z(), r)
                print "circle: cx: %s    cy: %s    r: %s"%(cx, cy, r)

            # calculate current absolute force vector in B (base) frame
            F_B = Fr_versor_B * Fr_value + Ft_versor_B * Ft_value
            
            # calculate new rotation of the gripper to align tool with force
            R_B_T = copy.deepcopy((self.T_B_W * self.T_W_T).M)

            normal_B = PyKDL.Frame(R_B_T) * PyKDL.Vector(0,0,1)
            alpha_norm = math.atan2(normal_B.y(), normal_B.x())
            alpha_force = math.atan2(F_B.y(), F_B.x())
            alpha_diff = self.fixAngleDiff(alpha_force-alpha_norm)
            if alpha_diff > 0.01:
                alpha += 0.01
            elif alpha_diff < -0.01:
                alpha -= 0.01
            else:
                alpha += alpha_diff

#            print "alpha_force: %s     alpha_norm: %s    alpha_diff: %s    alpha: %s"%(alpha_force, alpha_norm, alpha_diff, alpha)
#            print "Ft_value: %s"%(Ft_value)

            # calculate rotation frame
            T_E_Ed = PyKDL.Frame(PyKDL.Rotation.RotX(-alpha))
            R_B_Ed = copy.deepcopy((door.getAttribute("base").value * T_M_Ed * T_E_Ed).M)

            # calculate current absolute force vector in T (tool) frame FOR NEXT STEP
            R_B_Td = copy.deepcopy((PyKDL.Frame(R_B_Ed) * self.T_E_W * self.T_W_T).M)
            F_T = R_B_Td.Inverse() * F_B

            # white vector - force F_T
            self.publishVectorMarker(PyKDL.Vector(0,0,0),F_T*0.011, 5, 1,1,1, "right_arm_tool")
            # green vector - force F_B
            self.publishVectorMarker(self.T_B_W * self.T_W_T * PyKDL.Vector(0,0,0), self.T_B_W * self.T_W_T * PyKDL.Vector(0,0,0) + F_B*0.01, 2, 0,1,0, "torso_base")
            # light green vector - force Fr
            self.publishVectorMarker(self.T_B_W * self.T_W_T * PyKDL.Vector(0,0,0), self.T_B_W * self.T_W_T * PyKDL.Vector(0,0,0) + Fr_versor_B*Fr_value*0.01, 3, 0.3,1,0.3, "torso_base")
            # dark green vector - force Ft
            self.publishVectorMarker(self.T_B_W * self.T_W_T * PyKDL.Vector(0,0,0), self.T_B_W * self.T_W_T * PyKDL.Vector(0,0,0) + Ft_versor_B*Ft_value*0.01, 4, 0.3,1,0.3, "torso_base")

            # calculate spring offset from force and stiffness in T frame
            spring_T = PyKDL.Vector(F_T.x()/self.k_open.force.x, F_T.y()/self.k_open.force.y, F_T.z()/self.k_open.force.z)
            # calculate spring offset in B frame
            spring_B = R_B_Td * spring_T

            E_pt_dest = P_contact + spring_B

            E_pt_dest = PyKDL.Vector(E_pt_dest.x(), E_pt_dest.y(), self.pz)
            self.publishSinglePointMarker(E_pt_dest.x(), E_pt_dest.y(), E_pt_dest.z(), self.m_id, r=0.0, g=1.0, b=0.0)
            self.m_id += 1

            T_B_Wd = self.calculateMoveGripperPointToPose( self.E_pt, R_B_Ed, E_pt_dest )
            self.moveWrist2(T_B_Wd)
#            raw_input("Press Enter to continue...")
            self.moveWrist(T_B_Wd, 0.1, Wrench(Vector3(25,25,25), Vector3(5,5,5)))
            end_t = rospy.Time.now()+rospy.Duration(0.11)
            while rospy.Time.now()<end_t:
                self.getTransformationsForContact(100)            
                contact_time_diff = self.last_contact_time-rospy.Time.now()
                if self.hasContact(100) and contact_time_diff.to_sec() < 0.2:
                    T_B_C = self.T_B_W*self.T_W_E*self.T_E_F*self.T_F_C
                    P_contact = T_B_C*PyKDL.Vector(0,0,0)
                    self.addContact(P_contact)

                self.checkStopCondition(0.01)
                if self.emergency_stop_active:
                    self.failure_reason = "emergency_stop"
                    print "end: emergency stop"
                    return 0

                if not self.hasContact(50,print_on_false=True):
                    no_contact += 1
                else:
                    no_contact = 0
                if no_contact > 3:
#                    self.emergencyStop()
                    self.failure_reason = "lost_contact"
                    print "end: lost contact"
#                    rospy.sleep(1.0)
#                    return score

#            if abs(alpha_diff)<0.1:
            if alpha_diff<0.2:
#                door_position += 0.01 * 0.1
                door_position += 0.005 * 0.1

            arc = self.calculateArc()
            if abs(arc[0]) > abs(arc[1]):
                y1 = -2.0
                y2 = 2.0
                x1 = -(arc[1]*y1 + arc[2])/arc[0]
                x2 = -(arc[1]*y2 + arc[2])/arc[0]
            else:
                x1 = -2.0
                x2 = 2.0
                y1 = -(arc[0]*x1+arc[2])/arc[1]
                y2 = -(arc[0]*x2+arc[2])/arc[1]

            # white vector - force F_T
            self.publishVectorMarker(PyKDL.Vector(x1,y1,self.pz),PyKDL.Vector(x2,y2,self.pz), 6, 1,1,1, "torso_base")

            if len(self.px) > 0:
                self.publishVectorMarker(PyKDL.Vector(self.px[-1],self.py[-1],self.pz),PyKDL.Vector(self.px[-1]+arc[0],self.py[-1]+arc[1],self.pz), 7, 1,1,1, "torso_base")

            print "arc_state: %s   arc_len: %s"%(self.arc_state, arc[3])

            if state == 0:
                current_pos = door.getAttribute("base").value * door.getAttribute("handle").value + Ft_versor_B*door_position
                if door_position > 0.1:
                    state = 1
                    Fr_versor_old_B = copy.deepcopy(Fr_versor_B)
                    Ft_versor_old_B = copy.deepcopy(Ft_versor_B)

                    # change radial and tangent force versors and recalculate the forces
                    Fr_versor_B = copy.deepcopy( P_contact - PyKDL.Vector(cx, cy, self.pz) )
                    Fr_versor_B.Normalize()
                    # in the case we got circle with center on the wrong side
                    if PyKDL.dot(Fr_versor_B, Fr_versor_old_B) < 0:
                        Fr_versor_B = Fr_versor_B * (-1.0)
                    helper_versor = copy.deepcopy(Fr_versor_old_B * Ft_versor_old_B)
                    Ft_versor_B = copy.deepcopy(helper_versor*Fr_versor_B)
                    Ft_versor_B.Normalize()

                    Ft_value = copy.copy(PyKDL.dot(Ft_versor_B, F_B))
                    door_position = Ft_value/500.0

                    switch_pos = copy.deepcopy(P_contact)

            if state == 1:
                Fr_versor_old_B = copy.deepcopy(Fr_versor_B)
                Ft_versor_old_B = copy.deepcopy(Ft_versor_B)

                # change radial and tangent force versors and recalculate the forces
                Fr_versor_B = copy.deepcopy( P_contact - PyKDL.Vector(cx, cy, self.pz) )
                Fr_versor_B.Normalize()
                # in the case we got circle with center on the wrong side
                if PyKDL.dot(Fr_versor_B, Fr_versor_old_B) < 0:
                    Fr_versor_B = Fr_versor_B * (-1.0)
                helper_versor = copy.deepcopy(Fr_versor_old_B * Ft_versor_old_B)
                Ft_versor_B = copy.deepcopy(helper_versor*Fr_versor_B)
                Ft_versor_B.Normalize()

                switch_angle = math.atan2(switch_pos.y()-cy, switch_pos.x()-cx)
                current_angle = switch_angle + door_position/r
                current_pos = PyKDL.Vector(cx+r*math.cos(current_angle), cy+r*math.sin(current_angle), self.pz)

                angle = self.fixAngleDiff(init_angle-math.atan2(P_contact.y()-cy, P_contact.x()-cx))
#                print "angle: %s"%(angle)
                if (angle > 90.0/180.0*math.pi) or (angle < -90.0/180.0*math.pi):
                    print "end: angle: %s"%(angle)
                    break

            self.publishSinglePointMarker(current_pos.x(), current_pos.y(), current_pos.z(), self.m_id, r=0.2, g=0.2, b=1.0)
            self.m_id += 1

            if self.m_id > m_id_max:
                self.m_id = m_id_start

            Ft_value = 500.0 * PyKDL.dot( Ft_versor_B, current_pos - P_contact )
            if Ft_value < Fr_value*0.5:
                Ft_value = Fr_value*0.5

            continue

        return 0

######################
## door opening end ##
######################

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
                self.action = "next"
                self.index += 1
            if ch == 'r':
                self.action = "repeat"
            self.getTransformations()
            print "moving desired pose to current pose"
            self.emergency_stop_active = False
            self.moveWrist(self.T_B_W, 2.0, Wrench(Vector3(25,25,25), Vector3(5,5,5)))
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
        self.moveWrist( self.T_B_W, 2.0, Wrench(Vector3(10, 10, 10), Vector3(2, 2, 2)) )
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

        self.T_W_T = PyKDL.Frame(PyKDL.Vector(0.2,0,-0.05))    # tool transformation
        self.updateTool()

        # change the stiffness
        print "changing stiffness for door approach"
        self.moveImpedance(self.k_door, 2.0)
        self.checkStopCondition(2.0)

        raw_input("Press Enter to continue...")
        self.checkStopCondition()

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

        find_door_surface = False#True

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
                self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                self.checkStopCondition(3.0)

                T_B_Nd = door.getAttribute("base").value * P_e[i]
                T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
                self.moveWrist(T_B_Wd, 8.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))

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

                self.getTransformationsForContact(100)
                T_B_C = self.T_B_W * self.T_W_E * self.T_E_F * self.T_F_C
                P_door_surface.append( T_B_C * PyKDL.Vector(0, 0, 0) )

                T_B_Nd = door.getAttribute("base").value * P_s[i]
                T_B_Wd = T_B_Nd * self.T_N_F * self.T_F_E * self.T_E_W
                self.moveWrist(T_B_Wd, 2.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
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
        self.T_W_T = self.T_W_E * self.T_E_F * self.T_F_pt
        self.updateTool()

        self.getTransformations()

#        raw_input("Press Enter to continue...")

        # the orientation of E is set according to orientation of M (door.base)
        # the finger nail frame N is at position door.H_start

        # calculate rotation of gripper in door base frame M
        T_M_Ed = PyKDL.Frame( PyKDL.Rotation.RotZ(-math.pi/2.0) * PyKDL.Rotation.RotX(math.pi) )
        # calculate rotation of gripper in robot base frame B
        R_B_Ed = copy.deepcopy((door.getAttribute("base").value * T_M_Ed).M)

        T_B_Wd = self.calculateMoveGripperPointToPose( (self.T_E_F * self.T_F_N) * PyKDL.Vector(0,0,0), R_B_Ed, door.getAttribute("base").value * door.getAttribute("H_start").value )
        self.moveWrist(T_B_Wd, 4.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        self.checkStopCondition(4.0)

#        raw_input("Press Enter to continue...")

        # the orientation of E is set according to orientation of M (door.base)
        # the finger nail frame N is at position door.H_end
        T_B_Wd = self.calculateMoveGripperPointToPose( (self.T_E_F * self.T_F_N) * PyKDL.Vector(0,0,0), R_B_Ed, door.getAttribute("base").value * door.getAttribute("H_end").value )
        self.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))

        contact_found = False
        end_t = rospy.Time.now()+rospy.Duration(8.0)
        while rospy.Time.now()<end_t:
            self.checkStopCondition(0.05)
            self.getTransformationsForContact(100)
            if self.hasContact(100):
                contact_found = True
                self.stopArm()
                print "found contact"
                break
        if not contact_found:
            self.emergencyStop()
            rospy.sleep(1.0)
            return

        T_B_C = self.T_B_W * self.T_W_E * self.T_E_F * self.T_F_C
        T_M_B = door.getAttribute("base").value.Inverse()
        door.getAttribute("handle").value = T_M_B * T_B_C * PyKDL.Vector()
        door.getAttribute("pre_handle").value = T_M_B * T_B_C * PyKDL.Vector()

        door.getAttribute("handle").value += PyKDL.Vector(-0.01, 0.0, 0.0)
        door.getAttribute("pre_handle").value += PyKDL.Vector(0.10, 0.0, 0.05)

        print "handle: %s"%(door.getAttribute("handle").value)
        print "pre_handle: %s"%(door.getAttribute("pre_handle").value)

#        raw_input("Press Enter to continue...")

        self.exit_on_emergency_stop = False

        forces = [3.5, 7.0]
        r_a_tab = [ 0.04, 0.30 ]

        forces = [6.0, 6.1]
        r_a_tab = [ 0.05, 0.051 ]

        forces = [3.0, 4.0, 6.0, 8.0]
        r_a_tab = [ 0.02, 0.05, 0.2 ]

        grid_x, grid_y = np.mgrid[forces[0]:forces[-1]:400j, r_a_tab[0]:r_a_tab[-1]:400j]
        min_f_dist = 0.15*2.0
        min_r_dist = 0.05*2.0

        with open("experiments.txt", "a") as exfile:
            exfile.write("******** experiment series begin ********" + "\n")


        self.max_index = len(forces) * len(r_a_tab)

        used_forces = []
        used_r_a = []
        scores = []

        self.index = 0
        self.action = "next"
        # door opening loop
        while True:
            self.failure_reason = "unknown"

            self.clearDoorEstMarkers()

            if self.index < self.max_index:
                i = 0
                for forces_v in forces:
                    for r_a_v in r_a_tab:
                        if i == self.index:
                            self.force = forces_v
                            self.learning_r_a = r_a_v
                            self.learning_k_open_y = self.force / self.learning_r_a
                        i += 1
            elif self.action == "next":
                # linear nearest, cubic
                grid = griddata( (used_forces, used_r_a), scores, (grid_x, grid_y), 'linear')
                plt.xlabel('force')
                plt.ylabel('r_a')

                # find local minimum
                minimum = 10000.0
                coord = [-1, -1]
                for r in range(0, 400):
                    for f in range(0, 400):
                        if grid.T[r][f] < minimum:
                            # check if this point was wisited before
                            coord_u = [(f/399.0)*(forces[-1]-forces[0])+forces[0],  (r/399.0)*(r_a_tab[-1]-r_a_tab[0])+r_a_tab[0]]
                            cont = False
                            for i in range(0, len(used_forces)):
                                f_d = (used_forces[i]-coord_u[0])/min_f_dist
                                r_d = (used_r_a[i]-coord_u[1])/min_r_dist
                                if math.sqrt( f_d*f_d + r_d*r_d ) < 1.0:
                                    cont = True
                                    break
                            if cont:
                                continue
                            coord = [f, r]
                            minimum = grid.T[r][f]

                print [(coord[0]/399.0)*(forces[-1]-forces[0])+forces[0],  (coord[1]/399.0)*(r_a_tab[-1]-r_a_tab[0])+r_a_tab[0]]
                print minimum
                print coord

                self.force = (coord[0]/399.0)*(forces[-1]-forces[0])+forces[0]
                self.learning_r_a = (coord[1]/399.0)*(r_a_tab[-1]-r_a_tab[0])+r_a_tab[0]
                self.learning_k_open_y = self.force / self.learning_r_a

                plt.imshow(grid.T, extent=(forces[0],forces[-1],r_a_tab[0],r_a_tab[-1]), aspect='auto', origin='lower')
                plt.plot(used_forces, used_r_a, 'ro')

                plt.show()

            self.action = "next"
            self.r_a = copy.deepcopy( self.learning_r_a )
            self.k_open = copy.deepcopy( Wrench(Vector3(500.0, self.learning_k_open_y, 300.0), Vector3(300.0, 300.0, 300.0)) )

            print "parameters:"
            print "index: %s"%(self.index)
            print "force: %s"%(self.force)
            print "r_a: %s"%(self.learning_r_a)
            print "k_open_y: %s"%(self.learning_k_open_y)

            with open("experiments.txt", "a") as exfile:
                exfile.write("parameters:\n")
                exfile.write("index:"+str(self.index) + "\n")
                exfile.write("force:"+str(self.force) + "\n")
                exfile.write("r_a:"+str(self.learning_r_a) + "\n")
                exfile.write("k_open_y:"+str(self.learning_k_open_y) + "\n")

            self.moveImpedance(self.k_door2, 2.0)
            self.checkStopCondition(2.0)

            self.getTransformations()

#            raw_input("Press Enter to continue...")

#            T_B_Wd = self.calculateMoveGripperPointToPose( (self.T_E_F * self.T_F_N) * PyKDL.Vector(0,0,0), R_B_Ed, door.getAttribute("base").value * door.getAttribute("pre_handle").value )
#            self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
#            self.checkStopCondition(3.1)

#            if self.handleEmergencyStop():
#                continue

#            raw_input("Press Enter to continue...")

#            T_B_Wd = self.calculateMoveGripperPointToPose( (self.T_E_F * self.T_F_N) * PyKDL.Vector(0,0,0), R_B_Ed, door.getAttribute("base").value * door.getAttribute("handle").value )
#            self.moveWrist(T_B_Wd, 3.0, Wrench(Vector3(25,25,25), Vector3(4,4,4)))
#            self.checkStopCondition(3.1)

#            if self.handleEmergencyStop():
#                continue

            score_open = self.openTheDoor()

            cont = self.handleEmergencyStop()

            if self.action == "next":
                used_forces.append(copy.deepcopy(self.force))
                used_r_a.append(copy.deepcopy(self.learning_r_a))
                wrench_total = math.sqrt(self.wrench_max.force.x*self.wrench_max.force.x + self.wrench_max.force.y*self.wrench_max.force.y + self.wrench_max.force.z*self.wrench_max.force.z) + 10*math.sqrt(self.wrench_max.torque.x*self.wrench_max.torque.x + self.wrench_max.torque.y*self.wrench_max.torque.y + self.wrench_max.torque.z*self.wrench_max.torque.z)
                wrench_mean_total = math.sqrt(self.wrench_mean.force.x*self.wrench_mean.force.x + self.wrench_mean.force.y*self.wrench_mean.force.y + self.wrench_mean.force.z*self.wrench_mean.force.z) + 10*math.sqrt(self.wrench_mean.torque.x*self.wrench_mean.torque.x + self.wrench_mean.torque.y*self.wrench_mean.torque.y + self.wrench_mean.torque.z*self.wrench_mean.torque.z)
                scores.append(wrench_total+wrench_mean_total+score_open)
                self.printQualityMeasure(score_open)

            if cont:
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
    door.getAttribute("P1_start").value = PyKDL.Frame( rot, PyKDL.Vector(0.10, -0.26, 0.15) )
    door.getAttribute("P1_end").value   = PyKDL.Frame( rot, PyKDL.Vector(0.10, -0.26, -0.10) )
    door.getAttribute("P2_start").value = PyKDL.Frame( rot, PyKDL.Vector(0.0, 0.02, 0.15) )
    door.getAttribute("P2_end").value   = PyKDL.Frame( rot, PyKDL.Vector(0.0, 0.02, -0.10) )
    door.getAttribute("P3_start").value = PyKDL.Frame( rot, PyKDL.Vector(0.15, 0.02, 0.15) )
    door.getAttribute("P3_end").value   = PyKDL.Frame( rot, PyKDL.Vector(0.15, 0.02, -0.10) )

    # for small radius
#    door.getAttribute("H_start").value = PyKDL.Vector(0.10, -0.12, 0.04)
#    door.getAttribute("H_end").value   = PyKDL.Vector(-0.10, -0.12, 0.04)

    # for big radius
    door.getAttribute("H_start").value = PyKDL.Vector(0.0, -0.12, 0.04)
    door.getAttribute("H_end").value   = PyKDL.Vector(-0.20, -0.12, 0.04)

    door.getAttribute("hinge_pos").value   = PyKDL.Vector(0.20, 0.0, 0.0)

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

