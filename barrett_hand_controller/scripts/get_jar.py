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

jar_makrer_id=0
prefix="right"

def tupleToPose(t):
    return Pose(Point(t[0][0], t[0][1], t[0][2]), Quaternion(t[1][0], t[1][1], t[1][2], t[1][3]))

def PoseToTuple(p):
    return [p.position.x, p.position.y, p.position.z], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

def PoseToPosition(p):
    return [p.position.x, p.position.y, p.position.z, 1]

def moveArm(gripper_pose):

    global prefix
    global tf_listener
    global hand_arm_transform
    global tool
    global p
    global arm_pub

    gripper = gripper_pose

    real_gripper = tf_listener.lookupTransform('torso_base', prefix+'_HandPalmLink', rospy.Time(0))
#    print "real gripper pose: %s"%(real_gripper)

    real_tool = tf_listener.lookupTransform('torso_base', prefix+'_arm_7_link', rospy.Time(0))
    p = pm.toMsg(pm.fromMsg(gripper) * tool)
    dx = p.position.x-real_tool[0][0]
    dy = p.position.y-real_tool[0][1]
    dz = p.position.z-real_tool[0][2]
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    qw = quaternion_multiply(real_tool[1], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[3]
    angle = 2 * math.acos(qw)

    duration = length*20
    if angle*2>duration:
        duration = angle*2

    trj = CartesianTrajectory()
        
    trj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        
    trj.points.append(CartesianTrajectoryPoint(
    rospy.Duration(duration),
    p,
    Twist()))

    arm_pub.publish(trj)
    print "duration: %s"%(duration)
    print "pose: %s"%(p)
    return duration

def move_hand_client(prefix, f1, f2, f3, spread):
    rospy.wait_for_service('/' + prefix + '_hand/move_hand')
    try:
        move_hand = rospy.ServiceProxy('/' + prefix + '_hand/move_hand', BHMoveHand)
        resp1 = move_hand(f1, f2, f3, spread, 0.7, 0.7, 0.7, 0.7, 1000, 1000, 1000, 1000)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def alvarMarkerCallback(data):
    global jar_marker_pose
    global jar_marker_visible
    marker_count = len(data.markers)

    for i in range(0, marker_count):

        if data.markers[i].id == jar_makrer_id:
            jar_marker_pose = PoseToTuple(data.markers[i].pose.pose)
            jar_marker_visible = True

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

    jar_marker_visible = False
    jar_marker_pose = Pose()

    rospy.init_node('head_position', anonymous=True)
    print "Subscribing to tf"
    tf_listener = tf.TransformListener();
    br = tf.TransformBroadcaster()
    rospy.sleep(1)
    print "Subscribing to /ar_pose_marker"
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, alvarMarkerCallback)

    arm_pub = rospy.Publisher("/"+prefix+"_arm/trajectory", CartesianTrajectory)

    rate = 10.0
    period = 1.0/rate
    r = rospy.Rate(rate)	# 10 Hz
    while not rospy.is_shutdown():
        if jar_marker_visible:
            break        
        r.sleep()

    tf_listener.waitForTransform('torso_base', prefix+'_HandPalmLink', rospy.Time.now(), rospy.Duration(4.0))
    real_gripper = tf_listener.lookupTransform('torso_base', prefix+'_HandPalmLink', rospy.Time(0))

    real_gripper_mx = quaternion_matrix(real_gripper[1])
    real_gripper_mx[:3, 3] = real_gripper[0][:3]
    print "real gripper matrix:"
    print real_gripper_mx

    print "Found jar marker"

    tf_listener.waitForTransform('torso_base', prefix+'_arm_7_link', rospy.Time.now(), rospy.Duration(4.0))
    tool_msg = tf_listener.lookupTransform(prefix+'_HandPalmLink', prefix+'_arm_7_link', rospy.Time(0))
    tool = pm.fromTf(tool_msg)

    tf_listener.waitForTransform('torso_base', 'ar_marker_0', rospy.Time.now(), rospy.Duration(4.0))
    jar_marker = tf_listener.lookupTransform('torso_base', 'ar_marker_0', rospy.Time(0))
    jar_marker_mx = quaternion_matrix(jar_marker[1])
    jar_marker_mx[:3, 3] = jar_marker[0][:3]

    tf_listener.waitForTransform('torso_base', 'torso_link0', rospy.Time.now(), rospy.Duration(4.0))
    torso_link0 = tf_listener.lookupTransform('torso_base', 'torso_link0', rospy.Time(0))
    torso_link0_mx = quaternion_matrix(torso_link0[1])
    torso_link0_mx[:3, 3] = torso_link0[0][:3]

    # P is the point on jar's axis where the center of grip is
    P = jar_marker[0] - 0.05*jar_marker_mx[:3,2]
    # x axis of the gripper is equal to inverted z axis of jar's marker
    # z axis of the gripper is equal to y axis of torso_link0 (for right hand or inverse for left hand)
    if prefix == "right":
        grip_x = -jar_marker_mx[:3,2]
        grip_z = torso_link0_mx[:3,1]
    else:
        grip_x = jar_marker_mx[:3,2]
        grip_z = -torso_link0_mx[:3,1]

    # y axis of the gripper is the cross product of z and x axis
    grip_y = numpy.cross(grip_z, grip_x)

    # point G is gripper position
    G1 = P-grip_z*0.24
    G2 = P-grip_z*0.14

    gripper_mx = identity_matrix()
    gripper_mx[:3,0] = grip_x
    gripper_mx[:3,1] = grip_y
    gripper_mx[:3,2] = grip_z

    # move to pregrasp position
#        br.sendTransform(translation_from_matrix(jar_marker_mx), quaternion_from_matrix(jar_marker_mx), rospy.Time.now(), "jar", "torso_base")
    time = 0.0
    duration = moveArm(tupleToPose([G1,quaternion_from_matrix(gripper_mx)])) + 1
    while not rospy.is_shutdown():
        br.sendTransform(G1, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar1", "torso_base")
        br.sendTransform(G2, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar2", "torso_base")
        r.sleep()
        time += period
        if time > duration:
            break;

    move_hand_client(prefix, 30.0/180.0*numpy.pi, 30.0/180.0*numpy.pi, 30.0/180.0*numpy.pi, 0)

    # move to grasping position
    time = 0.0
    duration = moveArm(tupleToPose([G2,quaternion_from_matrix(gripper_mx)])) + 1
    while not rospy.is_shutdown():
        br.sendTransform(G1, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar1", "torso_base")
        br.sendTransform(G2, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar2", "torso_base")
        r.sleep()
        time += period
        if time > duration:
            break;

    # perform grip
    move_hand_client(prefix, 90.0/180.0*numpy.pi, 90.0/180.0*numpy.pi, 90.0/180.0*numpy.pi, 0)

    # wait some time
    time = 0.0
    duration = 6
    while not rospy.is_shutdown():
        br.sendTransform(G1, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar1", "torso_base")
        br.sendTransform(G2, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar2", "torso_base")
        r.sleep()
        time += period
        if time > duration:
            break;

    # release hand
    move_hand_client(prefix, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0.0/180.0*numpy.pi, 0)

    # return to pregrasp position
    time = 0.0
    duration = moveArm(tupleToPose([G1,quaternion_from_matrix(gripper_mx)])) + 1
    while not rospy.is_shutdown():
        br.sendTransform(G1, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar1", "torso_base")
        br.sendTransform(G2, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar2", "torso_base")
        r.sleep()
        time += period
        if time > duration:
            break;

    # return to starting position
    time = 0.0
    duration = moveArm(tupleToPose([translation_from_matrix(real_gripper_mx),quaternion_from_matrix(real_gripper_mx)])) + 1
    while not rospy.is_shutdown():
        br.sendTransform(G1, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar1", "torso_base")
        br.sendTransform(G2, quaternion_from_matrix(gripper_mx), rospy.Time.now(), "jar2", "torso_base")
        r.sleep()
        time += period
        if time > duration:
            break;

