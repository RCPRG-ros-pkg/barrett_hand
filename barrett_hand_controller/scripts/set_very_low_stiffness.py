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

from geometry_msgs.msg import Vector3, Wrench
from cartesian_trajectory_msgs.msg import CartesianImpedance, CartesianImpedanceGoal, CartesianImpedanceAction, CartesianImpedanceTrajectoryPoint
import actionlib
from actionlib_msgs.msg import *

def moveImpedance(k, t):
    action_impedance_goal = CartesianImpedanceGoal()
    action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
    action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
    rospy.Duration(t),
    CartesianImpedance(k,Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7)))))
    action_impedance_client.send_goal(action_impedance_goal)

if __name__ == '__main__':

    a = []
    for arg in sys.argv:
        a.append(arg)

    if (len(a) > 1) and ((a[1]=="left") or ("right")):
        prefix = a[1]
    else:
        print "Usage: %s prefix"%a[0]
        exit(0)

    rospy.init_node('impedance_lowerer')

    action_impedance_client = actionlib.SimpleActionClient("/" + prefix + "_arm/cartesian_impedance", CartesianImpedanceAction)
    action_impedance_client.wait_for_server()

    moveImpedance(Wrench(Vector3(100.0, 100.0, 100.0), Vector3(20.0, 20.0, 20.0)), 1.0)
    rospy.sleep(1.1)


