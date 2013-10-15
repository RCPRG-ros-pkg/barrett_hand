#!/usr/bin/env python
import roslib; roslib.load_manifest('barrett_hand_controller')

import sys
import rospy
import math

import std_msgs.msg
from barrett_hand_controller.msg import *
from barrett_hand_controller.srv import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

def get_pressure_sensors_info_client():
    rospy.wait_for_service('barrett_hand_controller/get_pressure_info')
    try:
        get_pressure_sensors_info = rospy.ServiceProxy('barrett_hand_controller/get_pressure_info', BHGetPressureInfo)
        resp1 = get_pressure_sensors_info()
        return resp1.info
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def callback(data):
    global pressure_info
    m = MarkerArray()

    frame_id = []
    frame_id.append("right_HandFingerOneKnuckleThreeLink")
    frame_id.append("right_HandFingerTwoKnuckleThreeLink")
    frame_id.append("right_HandFingerThreeKnuckleThreeLink")
    frame_id.append("right_HandPalmLink")

    for sens in range(0, 4):
        for i in range(0, 24):
            # calculate cross product: halfside1 x halfside2
            cx = pressure_info.sensor[sens].halfside1[i].y*pressure_info.sensor[sens].halfside2[i].z - pressure_info.sensor[sens].halfside1[i].z*pressure_info.sensor[sens].halfside2[i].y
            cy = pressure_info.sensor[sens].halfside1[i].z*pressure_info.sensor[sens].halfside2[i].x - pressure_info.sensor[sens].halfside1[i].x*pressure_info.sensor[sens].halfside2[i].z
            cz = pressure_info.sensor[sens].halfside1[i].x*pressure_info.sensor[sens].halfside2[i].y - pressure_info.sensor[sens].halfside1[i].y*pressure_info.sensor[sens].halfside2[i].x

            length = math.sqrt(cx*cx + cy*cy + cz*cz)

            scale = 0

            if sens == 0:
                scale = data.finger1_tip[i]/256.0
            elif sens == 1:
                scale = data.finger2_tip[i]/256.0
            elif sens == 2:
                scale = data.finger3_tip[i]/256.0
            else:
                scale = data.palm_tip[i]/256.0

            cx = cx/length*scale*0.01
            cy = cy/length*scale*0.01
            cz = cz/length*scale*0.01
            marker = Marker()
            marker.header.frame_id = frame_id[sens]
            marker.header.stamp = rospy.Time.now()
            marker.ns = frame_id[sens]
            marker.id = i
            marker.type = 0
            marker.action = 0
            marker.points.append(Point(pressure_info.sensor[sens].center[i].x,pressure_info.sensor[sens].center[i].y,pressure_info.sensor[sens].center[i].z))
            marker.points.append(Point(pressure_info.sensor[sens].center[i].x+cx, pressure_info.sensor[sens].center[i].y+cy, pressure_info.sensor[sens].center[i].z+cz))
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.001;
            marker.scale.y = 0.002;
            marker.scale.z = 0.0;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            m.markers.append(marker)

    global pub
    pub.publish(m)

def listener():
    rospy.init_node('hand_markers', anonymous=True)
    rospy.Subscriber("/barrett_hand_controller/BHPressureState", BHPressureState, callback)
    rospy.spin()

if __name__ == "__main__":
    print "Requesting pressure sensors info"
    pressure_info = get_pressure_sensors_info_client()
    print "sensor[0].center[0]: %s, %s, %s"%(pressure_info.sensor[0].center[0].x, pressure_info.sensor[0].center[0].y, pressure_info.sensor[0].center[0].z)

    pub = rospy.Publisher('visualization_marker', MarkerArray)

    listener()

