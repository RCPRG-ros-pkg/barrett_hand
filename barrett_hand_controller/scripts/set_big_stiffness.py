#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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
roslib.load_manifest('velma_controller')

import rospy

from geometry_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
import tf
import tf_conversions.posemath as pm
import PyKDL

def moveImpedance(k, t):
    global action_impedance_client
    action_impedance_goal = CartesianImpedanceGoal()
    action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
    action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
    rospy.Duration(t),
    CartesianImpedance(k,Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
    action_impedance_client.send_goal(action_impedance_goal)

def moveWrist( wrist_frame, tool_frame, t, max_wrench):
    global action_trajectory_client
    # we are moving the tool, so: T_B_Wd*T_W_T
    wrist_pose = pm.toMsg(wrist_frame*tool_frame)

    action_trajectory_goal = CartesianTrajectoryGoal()
    action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.01)
    action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
    rospy.Duration(t),
    wrist_pose,
    Twist()))
    action_trajectory_goal.wrench_constraint = max_wrench
    action_trajectory_client.send_goal(action_trajectory_goal)

def moveTool(tool_frame, t):
    global action_tool_client
    tool_pose = pm.toMsg(tool_frame)
    action_tool_goal = CartesianTrajectoryGoal()
    action_tool_goal.trajectory.header.stamp = rospy.Time.now()
    action_tool_goal.trajectory.points.append(CartesianTrajectoryPoint(
    rospy.Duration(t),
    tool_pose,
    Twist()))
    action_tool_client.send_goal(action_tool_goal)


if __name__ == '__main__':

    a = []
    for arg in sys.argv:
        a.append(arg)

    if (len(a) > 1) and ((a[1]=="left") or ("right")):
        prefix = a[1]
    else:
        print "Usage: %s prefix"%a[0]
        exit(0)

    rospy.init_node('impedance_riser')

    listener = tf.TransformListener();

    action_impedance_client = actionlib.SimpleActionClient("/" + prefix + "_arm/cartesian_impedance", CartesianImpedanceAction)
    action_impedance_client.wait_for_server()

    action_trajectory_client = actionlib.SimpleActionClient("/" + prefix + "_arm/cartesian_trajectory", CartesianTrajectoryAction)
    action_trajectory_client.wait_for_server()

    action_tool_client = actionlib.SimpleActionClient("/" + prefix + "_arm/tool_trajectory", CartesianTrajectoryAction)
    action_tool_client.wait_for_server()

    rospy.sleep(1.0)

    # save current wrist position
    listener.waitForTransform('torso_base', prefix+'_arm_7_link', rospy.Time.now(), rospy.Duration(4.0))
    pose = listener.lookupTransform('torso_base', prefix+'_arm_7_link', rospy.Time(0))
    T_B_W = pm.fromTf(pose)

    T_W_T = PyKDL.Frame()    # tool transformation
    print "setting the tool to %s relative to wrist frame"%(T_W_T)
    # move both tool position and wrist position - the gripper holds its position
    print "moving wrist"
    # we assume that during the initialization there are no contact forces, so we limit the wrench
    moveWrist( T_B_W, T_W_T, 2.0, Wrench(Vector3(20, 20, 20), Vector3(4, 4, 4)) )
    print "moving tool"
    moveTool( T_W_T, 2.0 )
    rospy.sleep(2.0)

    # change the stiffness
    print "changing stiffness for door approach"
    moveImpedance(Wrench(Vector3(800.0, 800.0, 800.0), Vector3(300.0, 300.0, 300.0)), 4.0)
    rospy.sleep(4.0)


