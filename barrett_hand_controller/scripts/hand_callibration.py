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

import roslib; roslib.load_manifest('barrett_hand_controller')

import sys
import rospy
import math

from ar_track_alvar.msg import *
import std_msgs.msg
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
from cartesian_trajectory_msgs.msg import *

import tf
from tf import *
from tf.transformations import *
from tf2_msgs.msg import *
import tf_conversions.posemath as pm
import numpy as np

def tupleToPose(t):
    return Pose(Point(t[0][0], t[0][1], t[0][2]), Quaternion(t[1][0], t[1][1], t[1][2], t[1][3]))

def PoseToTuple(p):
    return [p.position.x, p.position.y, p.position.z], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

def callback(data):
    global prefix
    global marker_id
    global marker_pose

    count = len(data.markers)
    for i in range(0, count):
        if data.markers[i].id == marker_id:
            marker_pose = data.markers[i].pose.pose
#            print "found marker id %s"%(data.markers[i].id)

def moveHand(mx, my, mz):
    global prefix
    global tool
    global arm_pub
    tf_listener.waitForTransform(prefix+'_arm_7_link', prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
    tf_listener.waitForTransform('torso_base', prefix+'_arm_7_link', rospy.Time.now(), rospy.Duration(4.0))

    real_gripper = tf_listener.lookupTransform('torso_base', prefix+'_HandPalmLink', rospy.Time(0))
    gr = tupleToPose(real_gripper)
    gr.position.x += mx
    gr.position.y += my
    gr.position.z += mz
#    gr = Pose(Point(gripper[0][0]+dx, gripper[0][1]+dy, gripper[0][2]+dz), Quaternion(gripper[1][0], gripper[1][1], gripper[1][2], gripper[1][3]))
    real_tool = tf_listener.lookupTransform('torso_base', prefix+'_arm_7_link', rospy.Time(0))
    p = pm.toMsg(pm.fromMsg(gr) * tool)
    dx = p.position.x-real_tool[0][0]
    dy = p.position.y-real_tool[0][1]
    dz = p.position.z-real_tool[0][2]
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    qw = quaternion_multiply(real_tool[1], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[3]
    angle = 2 * math.acos(qw)

    duration = length*20
    if angle*2>duration:
        duration = angle*2

    print "moveHand(%s, %s, %s) in %ss"%(mx, my, mz, duration)

    trj = CartesianTrajectory()

    trj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

    trj.points.append( CartesianTrajectoryPoint(rospy.Duration(duration), p, Twist()) )

    arm_pub.publish(trj)

    return duration

def getToolPose():
    global prefix
    real_tool = tf_listener.lookupTransform('torso_base', prefix+'_arm_7_link', rospy.Time(0))
    return tupleToPose(real_tool)

if __name__ == "__main__":
    a = []
    for arg in sys.argv:
        a.append(arg)

    if len(a) !=2:
        print "Usage: %s prefix"%a[0]
        exit(1)

    if a[1] == "left":
        prefix = a[1]
        marker_id = 2
    elif a[1] == "right":
        prefix = a[1]
        marker_id = 1
    else:
        print "Usage: %s prefix"%a[0]
        exit(0)

    marker_pose = Pose([0,0,0],[0,0,1,0])

    rospy.init_node('hand_callibration', anonymous=True)

    tf_listener = tf.TransformListener();
    rospy.sleep(1)

    tf_listener.waitForTransform(prefix+'_HandPalmLink', prefix+'_arm_7_link', rospy.Time.now(), rospy.Duration(4.0))
    tool_msg = tf_listener.lookupTransform(prefix+'_HandPalmLink', prefix+'_arm_7_link', rospy.Time(0))
    tool = pm.fromTf(tool_msg)

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback)
    arm_pub = rospy.Publisher("/"+prefix+"_arm/trajectory", CartesianTrajectory)

    rate = 10.0
    period = 1.0/rate
    duration = 1.0
    step = 0
    r = rospy.Rate(rate)	# 10 Hz
    while not rospy.is_shutdown():
        if duration<=-1:
            if step == 0:
                real_c = getToolPose()
                marker_c = marker_pose
                duration = moveHand(0.1, 0, 0);
                step += 1
            elif step == 1:
                real_x = getToolPose()
                marker_x = marker_pose
                duration = moveHand(-0.1, 0.1, 0);
                step += 1
#            elif step == 2:
#                duration = moveHand(0, 0.1, 0);
#                step += 1
            elif step == 2:
                real_y = getToolPose()
                marker_y = marker_pose
                duration = moveHand(0, -0.1, 0.1);
                step += 1
#            elif step == 3:
#                duration = moveHand(0, 0, 0.1);
#                step += 1
            elif step == 3:
                real_z = getToolPose()
                marker_z = marker_pose
                duration = moveHand(0, 0, -0.1);
                step += 1
            elif step == 4:
                step += 1
                break

        duration -= period
        r.sleep()

    marker_x.position.x -= marker_c.position.x
    marker_x.position.y -= marker_c.position.y
    marker_x.position.z -= marker_c.position.z
    marker_y.position.x -= marker_c.position.x
    marker_y.position.y -= marker_c.position.y
    marker_y.position.z -= marker_c.position.z
    marker_z.position.x -= marker_c.position.x
    marker_z.position.y -= marker_c.position.y
    marker_z.position.z -= marker_c.position.z

    marker_c = PoseToTuple(marker_c)
    transform = [0,0,0], quaternion_inverse(marker_c[1])
    transform = pm.fromTf(transform)

    m_x = unit_vector(PoseToTuple(pm.toMsg(transform*pm.fromMsg(marker_x)))[0])
    m_y = unit_vector(PoseToTuple(pm.toMsg(transform*pm.fromMsg(marker_y)))[0])
    m_z = unit_vector(PoseToTuple(pm.toMsg(transform*pm.fromMsg(marker_z)))[0])

    print "marker:"
    print m_x
    print m_y
    print m_z

    real_x.position.x -= real_c.position.x
    real_x.position.y -= real_c.position.y
    real_x.position.z -= real_c.position.z
    real_y.position.x -= real_c.position.x
    real_y.position.y -= real_c.position.y
    real_y.position.z -= real_c.position.z
    real_z.position.x -= real_c.position.x
    real_z.position.y -= real_c.position.y
    real_z.position.z -= real_c.position.z

    real_c = PoseToTuple(real_c)
    transform = [0,0,0], quaternion_inverse(real_c[1])
    transform = pm.fromTf(transform)

    r_x = unit_vector(PoseToTuple(pm.toMsg(transform*pm.fromMsg(real_x)))[0])
    r_y = unit_vector(PoseToTuple(pm.toMsg(transform*pm.fromMsg(real_y)))[0])
    r_z = unit_vector(PoseToTuple(pm.toMsg(transform*pm.fromMsg(real_z)))[0])

    print "tool:"
    print r_x
    print r_y
    print r_z

    print "angle between marker x,y: %s deg."%(180.0*math.acos(numpy.dot(m_x, m_y))/np.pi)
    print "angle between marker y,z: %s deg."%(180.0*math.acos(numpy.dot(m_y, m_z))/np.pi)
    print "angle between marker z,x: %s deg."%(180.0*math.acos(numpy.dot(m_z, m_x))/np.pi)

    print "angle between tool x,y: %s deg."%(180.0*math.acos(numpy.dot(r_x, r_y))/np.pi)
    print "angle between tool y,z: %s deg."%(180.0*math.acos(numpy.dot(r_y, r_z))/np.pi)
    print "angle between tool z,x: %s deg."%(180.0*math.acos(numpy.dot(r_z, r_x))/np.pi)

    m_z = unit_vector(numpy.cross(m_x, m_y))
    m_y = unit_vector(numpy.cross(m_z, m_x))
    r_z = unit_vector(numpy.cross(r_x, r_y))
    r_y = unit_vector(numpy.cross(r_z, r_x))

    print "making vectors orthogonal..."
    print "angle between marker x,y: %s deg."%(180.0*math.acos(numpy.dot(m_x, m_y))/np.pi)
    print "angle between marker y,z: %s deg."%(180.0*math.acos(numpy.dot(m_y, m_z))/np.pi)
    print "angle between marker z,x: %s deg."%(180.0*math.acos(numpy.dot(m_z, m_x))/np.pi)

    print "angle between tool x,y: %s deg."%(180.0*math.acos(numpy.dot(r_x, r_y))/np.pi)
    print "angle between tool y,z: %s deg."%(180.0*math.acos(numpy.dot(r_y, r_z))/np.pi)
    print "angle between tool z,x: %s deg."%(180.0*math.acos(numpy.dot(r_z, r_x))/np.pi)

    m = np.array([m_x, m_y, m_z])
    r = np.array([r_x, r_y, r_z])

    print "marker matrix:"
    print m

    print "tool matrix:"
    print r
    r_t = r.transpose()

    print "tool transposed:"
    print r_t

#    diff2 = np.dot(r_t,m)
#    print "r_t*m"
#    print diff2

    diff = np.dot(r_t,m)
    print "r_t*m"
    print diff

    transl = diff.transpose().dot(np.array([0, 0, 0.025]))

    diff = numpy.array((
        (diff[0][0], diff[0][1],     diff[0][2], 0.0),
        (diff[1][0], diff[1][1],     diff[1][2], 0.0),
        (diff[2][0], diff[2][1],     diff[2][2], 0.0),
        (       0.0,        0.0,            0.0, 1.0)
        ))

    print "diff:"
    print diff

    diff_q = quaternion_from_matrix(diff)

    print "tool_marker_%s_position = %s"%(prefix, transl)
    print "tool_marker_%s_orientation = %s"%(prefix, diff_q)

