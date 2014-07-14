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

import std_msgs.msg
import ar_track_alvar.msg
from ar_track_alvar.msg import *
#from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray
#from geometry_msgs.msg import Point
#from sensor_msgs.msg import Image

import tf
from tf import *
from tf.transformations import * 
#import tf_conversions.posemath as pm
from tf2_msgs.msg import *

from collections import deque

left_makrer_id=2
right_makrer_id=1

#tool_marker_left_position = [ 0.00013558,  0.00033355,  0.02499741]
#tool_marker_left_orientation = [ 0.00265232,  0.00669491,  0.99993464, -0.00887976]
tool_marker_left_position = [ -2.72290709e-05,   5.56445479e-04,   2.49937918e-02]
tool_marker_left_orientation = [ -6.22007822e-04,   1.11255436e-02,   9.99913731e-01,  -6.95451356e-03]

#tool_marker_right_position = [ 0.00038019,  0.0001039,   0.02499689]
#tool_marker_right_orientation = [  7.60508486e-03,   2.07433971e-03,   9.99968812e-01,   4.84539739e-04]
tool_marker_right_position = [  5.31013495e-05,  -1.31241002e-04,   2.49995991e-02]
tool_marker_right_orientation = [ 0.00104553, -0.00263145,  0.99997629,  0.00627708]

def PoseToTuple(p):
    return [p.position.x, p.position.y, p.position.z], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

def PoseToPosition(p):
    return [p.position.x, p.position.y, p.position.z, 1]

def alvarMarkerCallback(data):
    global br
    global tf_listener
    global tool_marker_left
    global tool_marker_right
    global cam
    global marker_visible
    marker_count = len(data.markers)

    for i in range(0, marker_count):

        if (data.markers[i].id == left_makrer_id) or (data.markers[i].id == right_makrer_id):
            prefix = "left"
            tool_marker = tool_marker_left
            if data.markers[i].id == right_makrer_id:
                prefix = "right"
                tool_marker = tool_marker_right

            pose = data.markers[i].pose.pose
            try:
#                tf_listener.waitForTransform('torso_link2', prefix+'_arm_7_link', rospy.Time.now(), rospy.Duration(1.0))
                torso_tool_tf = tf_listener.lookupTransform('torso_link2', prefix+'_arm_7_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return

            pose_t = PoseToTuple(pose)
            cam = quaternion_matrix(pose_t[1])
            cam[:3, 3] = PoseToPosition(pose)[:3]
            cam = inverse_matrix(cam)

            torso_tool = quaternion_matrix(torso_tool_tf[1])
            torso_tool[:3,3] = (torso_tool_tf[0])[:3]

            cam = numpy.dot(tool_marker, cam)
            cam = numpy.dot(torso_tool, cam)

            marker_visible = True

if __name__ == "__main__":
    a = []
    for arg in sys.argv:
        a.append(arg)

    tool_marker_left = quaternion_matrix(tool_marker_left_orientation)
    tool_marker_left[:3, 3] = tool_marker_left_position[:3]

    tool_marker_right = quaternion_matrix(tool_marker_right_orientation)
    tool_marker_right[:3, 3] = tool_marker_right_position[:3]

    cam = identity_matrix()
    marker_visible = False

    rospy.init_node('head_position', anonymous=True)
    print "Subscribing to tf"
    tf_listener = tf.TransformListener();
    br = tf.TransformBroadcaster()
    print "Subscribing to /ar_pose_marker"
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, alvarMarkerCallback)

    print "Do not move the camera!"
    print "Please show me the wrist marker..."

    queue_o = deque()
    queue_t = deque()

    steps = 30
#    rospy.spin()
    rate = 10.0
    r = rospy.Rate(rate)	# 10 Hz
    length = 0
    while not rospy.is_shutdown():
#        print "loop: %s"%(marker_visible)
        if marker_visible:
            marker_visible = False
            orientation = quaternion_from_matrix(cam)
#            euler = euler_from_matrix(cam)
            translation = translation_from_matrix(cam)
            queue_o.append(orientation)#numpy.array(euler))
            queue_t.append(translation)
            length += 1
            if length>steps:
                queue_o.popleft()
                queue_t.popleft()
                length -= 1

            if length == steps:
                mean_o = reduce(lambda x, y: x+y, queue_o)
                mean_t = reduce(lambda x, y: x+y, queue_t)
                mean_o[0] /= 1.0*steps
                mean_o[1] /= 1.0*steps
                mean_o[2] /= 1.0*steps
                mean_o[3] /= 1.0*steps
                mean_t[0] /= 1.0*steps
                mean_t[1] /= 1.0*steps
                mean_t[2] /= 1.0*steps
                variation_o = 0
                variation_t = 0
                for o in queue_o:
                    variation_o = (o[0]-mean_o[0])*(o[0]-mean_o[0]) + (o[1]-mean_o[1])*(o[1]-mean_o[1]) + (o[2]-mean_o[2])*(o[2]-mean_o[2]) + (o[3]-mean_o[3])*(o[3]-mean_o[3])
                for t in queue_t:
                    variation_t = (t[0]-mean_t[0])*(t[0]-mean_t[0]) + (t[1]-mean_t[1])*(t[1]-mean_t[1]) + (t[2]-mean_t[2])*(t[2]-mean_t[2])
                if (variation_o<0.000001) and (variation_t<0.000001):
                    print "o: %s   t: %s"%(variation_o, variation_t)
                    break
#            math.sqrt(numpy.dot(euler,euler))
#            math.sqrt(numpy.dot(translation,translation))
        
        r.sleep()

    while not rospy.is_shutdown():

        br.sendTransform(mean_t, mean_o, rospy.Time.now(), "camera", "torso_link2")
#        br.sendTransform(translation_from_matrix(cam), quaternion_from_matrix(cam), rospy.Time.now(), "camera", "torso_link2")
        r.sleep()

