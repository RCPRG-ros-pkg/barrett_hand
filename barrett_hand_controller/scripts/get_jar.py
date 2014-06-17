#!/usr/bin/env python
import roslib; roslib.load_manifest('barrett_hand_controller')

import sys
import rospy
import math

import std_msgs.msg
import ar_track_alvar.msg
from ar_track_alvar.msg import *
#from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import *
#from sensor_msgs.msg import Image
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

from cartesian_trajectory_msgs.msg import *

from collections import deque

import actionlib
from actionlib_msgs.msg import *

import PyKDL

import copy

jar_makrer_id=0
prefix="right"

class JarGripper:
    """
Class for gripping the jar with velma robot.
"""

    def tupleToPose(self, t):
        return Pose(Point(t[0][0], t[0][1], t[0][2]), Quaternion(t[1][0], t[1][1], t[1][2], t[1][3]))

    def PoseToTuple(self, p):
        return [p.position.x, p.position.y, p.position.z], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

    def PoseToPosition(self, p):
        return [p.position.x, p.position.y, p.position.z, 1]

    def moveWrist(self, wrist_frame, t, max_wrench):
        # we are moving the tool, so: T_B_Wd*T_W_T
        wrist_pose = pm.toMsg(wrist_frame*self.T_W_T)
        self.br.sendTransform(self.PoseToTuple(wrist_pose)[0], self.PoseToTuple(wrist_pose)[1], rospy.Time.now(), "dest", "torso_base")

        action_trajectory_goal = CartesianTrajectoryGoal()
        action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.01)
        action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(t),
        wrist_pose,
        Twist()))
        action_trajectory_goal.wrench_constraint = max_wrench
        self.current_max_wrench = max_wrench
        self.action_trajectory_client.send_goal(action_trajectory_goal)

    def moveWristTraj(self, wrist_frames, times, max_wrench):
        # we are moving the tool, so: T_B_Wd*T_W_T
        action_trajectory_goal = CartesianTrajectoryGoal()
        action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.01)

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

    def moveImpedance(self, k, t):
        action_impedance_goal = CartesianImpedanceGoal()
        action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
        rospy.Duration(t),
        CartesianImpedance(k,Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
        self.action_impedance_client.send_goal(action_impedance_goal)

    def move_hand_client(self, prefix, q):
        rospy.wait_for_service('/' + self.prefix + '_hand/move_hand')
        try:
            move_hand = rospy.ServiceProxy('/' + self.prefix + '_hand/move_hand', BHMoveHand)
            resp1 = move_hand(q[0], q[1], q[2], q[3], 1.2, 1.2, 1.2, 1.2, 2000, 2000, 2000, 2000)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def alvarMarkerCallback(self, data):
        marker_count = len(data.markers)

        for i in range(0, marker_count):

            if data.markers[i].id == jar_makrer_id:
                self.jar_marker_pose = self.PoseToTuple(data.markers[i].pose.pose)
                self.jar_marker_visible = True

    def stopArm(self):
        if self.action_trajectory_client.gh:
            self.action_trajectory_client.cancel_goal()
        if self.action_tool_client.gh:
            self.action_tool_client.cancel_goal()

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
#            if self.wrench_emergency_stop:
#                self.emergencyStop()
#                print "too big wrench"
#                self.failure_reason = "too_big_wrench"
#                rospy.sleep(1.0)
#                if self.exit_on_emergency_stop:
#                    self.emergencyExit()

            if (self.action_trajectory_client.gh) and ((self.action_trajectory_client.get_state()==GoalStatus.REJECTED) or (self.action_trajectory_client.get_state()==GoalStatus.ABORTED)):
                state = self.action_trajectory_client.get_state()
                result = self.action_trajectory_client.get_result()
                self.emergencyStop()
#                print "emergency stop: traj_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "too_big_wrench_trajectory"
                rospy.sleep(1.0)
                if self.exit_on_emergency_stop:
                    self.emergencyExit()

            if (self.action_tool_client.gh) and ((self.action_tool_client.get_state()==GoalStatus.REJECTED) or (self.action_tool_client.get_state()==GoalStatus.ABORTED)):
                state = self.action_tool_client.get_state()
                result = self.action_tool_client.get_result()
                self.emergencyStop()
#                print "emergency stop: tool_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "too_big_wrench_tool"
                rospy.sleep(1.0)
                if self.exit_on_emergency_stop:
                    self.emergencyExit()
            rospy.sleep(0.01)
        return self.emergency_stop_active

    def checkForExit(self):
        if rospy.is_shutdown():
            exit(0)

    def __init__(self, pref):
        self.prefix = pref
        self.jar_marker_visible = False
        self.jar_marker_pose = Pose()

        self.q_start = (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 180.0/180.0*numpy.pi) 
        self.q_end = (120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 180.0/180.0*numpy.pi) 
        self.q_pregrip = (0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi) 
        self.q_grip = (120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 0.0/180.0*numpy.pi) 
        self.k_error = Wrench(Vector3(10.0, 10.0, 10.0), Vector3(2.0, 2.0, 2.0))
        self.k_jar = Wrench(Vector3(400.0, 400.0, 400.0), Vector3(200.0, 200.0, 200.0))

        self.T_W_T = PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.0))    # zero tool transformation

        self.exit_on_emergency_stop = True
        self.emergency_stop_active = False

        print "Subscribing to tf"
        self.listener = tf.TransformListener();
        self.br = tf.TransformBroadcaster()
        print "Subscribing to /ar_pose_marker"
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvarMarkerCallback)

        self.action_trajectory_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/cartesian_trajectory", CartesianTrajectoryAction)
        self.action_trajectory_client.wait_for_server()

        self.action_tool_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/tool_trajectory", CartesianTrajectoryAction)
        self.action_tool_client.wait_for_server()

        self.action_impedance_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/cartesian_impedance", CartesianImpedanceAction)
        self.action_impedance_client.wait_for_server()

    def getTransformations(self):
        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))
        self.T_B_W = pm.fromTf(pose)

#        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
#        self.T_E_F = pm.fromTf(pose)
#        self.T_F_E = self.T_E_F.Inverse()

    def spin(self):
        rospy.sleep(1.0)

        rate = 10.0
        period = 1.0/rate
        r = rospy.Rate(rate)	# 10 Hz
        while not rospy.is_shutdown():
            if self.jar_marker_visible:
                break        
            r.sleep()

        print "Found jar marker"

        # get door marker absolute position
        self.listener.waitForTransform('torso_base', 'ar_marker_0', rospy.Time.now(), rospy.Duration(4.0))
        door_marker = self.listener.lookupTransform('torso_base', 'ar_marker_0', rospy.Time(0))

        print door_marker
        self.T_B_M = pm.fromTf(door_marker)

        self.listener.waitForTransform(self.prefix+'_arm_7_link', self.prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
        pose = self.listener.lookupTransform(self.prefix+'_arm_7_link', self.prefix+'_HandPalmLink', rospy.Time(0))
        self.T_W_E = pm.fromTf(pose)
        self.T_E_W = self.T_W_E.Inverse()

        # start with very low stiffness
        print "setting stiffness to very low value"
        self.moveImpedance(self.k_error, 0.5)
        self.checkStopCondition(0.5)

        raw_input("Press Enter to continue...")
        self.checkStopCondition()

        self.getTransformations()

        print "setting the tool to %s relative to wrist frame"%(self.T_W_T)
        # move both tool position and wrist position - the gripper holds its position
        print "moving wrist"
        # we assume that during the initialization there are no contact forces, so we limit the wrench
        self.moveWrist( self.T_B_W, 2.0, Wrench(Vector3(5, 5, 5), Vector3(2, 2, 2)) )
        print "moving tool"
        self.moveTool( self.T_W_T, 2.0 )
        self.checkStopCondition(2.0)

        # change the stiffness
        print "changing stiffness for door approach"
        self.moveImpedance(self.k_jar, 2.0)
        self.checkStopCondition(2.0)


        # straighten fingers
        self.move_hand_client(self.prefix, self.q_start)

        rospy.sleep(2.0)

        if self.prefix == "right":
            self.T_B_W.M = PyKDL.Rotation.RotZ(math.pi/4.0)
        else:
            self.T_B_W.M = PyKDL.Rotation.RotZ(-math.pi/4.0)
        self.moveWrist( self.T_B_W, 3.0, Wrench(Vector3(15, 15, 15), Vector3(4, 4, 4)) )
        self.checkStopCondition(3.0)

        self.getTransformations()

        T_W_B = self.T_B_W.Inverse()
        T_E_M = self.T_E_W * T_W_B * self.T_B_M
        T_E_M.p = PyKDL.Vector(0.10, 0.0, 0.30)
        T_M_E = T_E_M.Inverse()
        
        print "moving close to jar"
        T_B_Wd = self.T_B_M * T_M_E * self.T_E_W
        T_B_Wd_table_pre = copy.deepcopy(T_B_Wd)
        self.moveWrist( T_B_Wd, 3.0, Wrench(Vector3(15, 15, 15), Vector3(4, 4, 4)) )
        self.checkStopCondition(3.0)

        self.move_hand_client(self.prefix, self.q_pregrip)
        self.checkStopCondition(2.0)

        self.getTransformations()

        T_W_B = self.T_B_W.Inverse()
        T_E_M = self.T_E_W * T_W_B * self.T_B_M
        T_E_M.p = PyKDL.Vector(-0.11, 0.0, 0.12)
        T_M_E = T_E_M.Inverse()
        
        print "moving closer to jar"
        T_B_Wd = self.T_B_M * T_M_E * self.T_E_W
        T_B_Wd_table = copy.deepcopy(T_B_Wd)
        self.moveWrist( T_B_Wd, 3.0, Wrench(Vector3(15, 15, 15), Vector3(4, 4, 4)) )
        self.checkStopCondition(3.0)

        self.move_hand_client(self.prefix, self.q_grip)
        self.checkStopCondition(3.0)

        raw_input("Press Enter to continue...")

        print "moving somewhere..."
        T_B_Wd = PyKDL.Frame(PyKDL.Rotation.RotZ(0.0), PyKDL.Vector(0.5, -0.3, 1.1))
        self.moveWrist( T_B_Wd, 4.0, Wrench(Vector3(15, 15, 15), Vector3(4, 4, 4)) )
        self.checkStopCondition(4.0)

        raw_input("Press Enter to continue...")

        self.moveWrist( T_B_Wd_table, 4.0, Wrench(Vector3(15, 15, 15), Vector3(4, 4, 4)) )
        self.checkStopCondition(4.0)

        raw_input("Press Enter to continue...")

        self.move_hand_client(self.prefix, self.q_pregrip)
        self.checkStopCondition(2.0)

        raw_input("Press Enter to continue...")

        self.moveWrist( T_B_Wd_table_pre, 4.0, Wrench(Vector3(15, 15, 15), Vector3(4, 4, 4)) )
        self.checkStopCondition(4.0)

        raw_input("Press Enter to continue...")

        self.move_hand_client(self.prefix, self.q_start)
        self.checkStopCondition(2.0)

        print "moving somewhere..."
        T_B_Wd = PyKDL.Frame(PyKDL.Rotation.RotZ(0.0), PyKDL.Vector(0.5, -0.3, 1.1))
        self.moveWrist( T_B_Wd, 4.0, Wrench(Vector3(15, 15, 15), Vector3(4, 4, 4)) )
        self.checkStopCondition(2.0)

        self.move_hand_client(self.prefix, self.q_end)
        self.checkStopCondition(2.0)

if __name__ == "__main__":
    a = []
    for arg in sys.argv:
        a.append(arg)

    if len(a) !=2:
        print "Usage: %s prefix"%a[0]
        exit(1)

    if (a[1] == "left") or (a[1] == "right"):
        prefix = a[1]
    else:
        print "Usage: %s prefix"%a[0]
        exit(1)

    rospy.init_node('head_position', anonymous=True)
    jarHolder = JarGripper(a[1])
    jarHolder.spin()

