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
from sensor_msgs.msg import Image

import tf
from tf import *
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import *

def get_pressure_sensors_info_client():
    rospy.wait_for_service('barrett_hand_controller/get_pressure_info')
    try:
        get_pressure_sensors_info = rospy.ServiceProxy('barrett_hand_controller/get_pressure_info', BHGetPressureInfo)
        resp1 = get_pressure_sensors_info()
        return resp1.info
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def convertToRGB(value):
    r = 0
    g = 0
    b = 0
    if value<256:
        b = 255
        g = value
    elif value<512:
        b = 255-(value-256)
        g = 255
    elif value<768:
        g = 255
        r = (value-512)
    elif value<1024:
        r = 255
        g = 255-(value-768)
    else:
        r = 255
        g = 0
        b = 0
    result = []
    result.append(r)
    result.append(g)
    result.append(b)
    return result

def callback(data):
    global pressure_info
    m = MarkerArray()

    frame_id = []
    frame_id.append("right_HandFingerOneKnuckleThreeLink")
    frame_id.append("right_HandFingerTwoKnuckleThreeLink")
    frame_id.append("right_HandFingerThreeKnuckleThreeLink")
    frame_id.append("right_HandPalmLink")


    tran = []
    rotat = []
    (transl, rot) = tf_listener.lookupTransform('right_HandPalmLink', frame_id[0], rospy.Time(0))
    tran.append(transl)
    rotat.append(rot)
    (transl, rot) = tf_listener.lookupTransform('right_HandPalmLink', frame_id[1], rospy.Time(0))
    tran.append(transl)
    rotat.append(rot)
    (transl, rot) = tf_listener.lookupTransform('right_HandPalmLink', frame_id[2], rospy.Time(0))
    tran.append(transl)
    rotat.append(rot)

    sumx = 0
    sumy = 0
    sumz = 0
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

            if sens < 3:
                a = rotat[sens][3]
                b = rotat[sens][0]
                c = rotat[sens][1]
                d = rotat[sens][2]
                gx = (a*a+b*b-c*c-d*d)*cx + 2*(b*c-a*d)*cy + 2*(b*d+a*c)*cz
                gy = 2*(b*c+a*d)*cx + (a*a-b*b+c*c-d*d)*cy + 2*(c*d-a*b)*cz
                gz = 2*(b*d-a*c)*cx + 2*(c*d+a*b)*cy + (a*a-b*b-c*c+d*d)*cz
                sumx += gx
                sumy += gy
                sumz += gz
            else:
                sumx += cx
                sumy += cy
                sumz += cz

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

    marker = Marker()
    marker.header.frame_id = frame_id[3]
    marker.header.stamp = rospy.Time.now()
    marker.ns = frame_id[3]
    marker.id = i
    marker.type = 0
    marker.action = 0
    marker.points.append(Point(0,0,0.15))
    marker.points.append(Point(1*sumx, 1*sumy, 1*sumz+0.15))
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
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    m.markers.append(marker)

    pub.publish(m)

#  palm   f1  f2  f3
#         xxx xxx xxx
#         xxx xxx xxx 
#  xxxxx  xxx xxx xxx
# xxxxxxx xxx xxx xxx
# xxxxxxx xxx xxx xxx
#  xxxxx  xxx xxx xxx
#         xxx xxx xxx
#         xxx xxx xxx

    global tactileImagepub
    im = Image()
    im.height = 8
    im.width = 19
    im.encoding = "rgb8"
    im.is_bigendian = 0 #False
    im.step = im.width*3
    im.data = [0]*(im.step*im.height)
    for y in range(0, 8):
        for x in range(0, 3):
            xim = 8+x
            yim = im.height-1-y
            value = convertToRGB(int(data.finger1_tip[y*3+x]/2))
            im.data[(yim*im.width + xim)*3+0] = value[0]
            im.data[(yim*im.width + xim)*3+1] = value[1]
            im.data[(yim*im.width + xim)*3+2] = value[2]

    for y in range(0, 8):
        for x in range(0, 3):
            xim = 12+x
            yim = im.height-1-y
            value = convertToRGB(int(data.finger2_tip[y*3+x]/2))
            im.data[(yim*im.width + xim)*3+0] = value[0]
            im.data[(yim*im.width + xim)*3+1] = value[1]
            im.data[(yim*im.width + xim)*3+2] = value[2]

    for y in range(0, 8):
        for x in range(0, 3):
            xim = 16+x
            yim = im.height-1-y
            value = convertToRGB(int(data.finger3_tip[y*3+x]/2))
            im.data[(yim*im.width + xim)*3+0] = value[0]
            im.data[(yim*im.width + xim)*3+1] = value[1]
            im.data[(yim*im.width + xim)*3+2] = value[2]

    i = 0
    y = 0
    for x in range(0, 5):
        xim = 1+x
        yim = im.height-1-(2+y)
        value = convertToRGB(int(data.palm_tip[i]/2))
        im.data[(yim*im.width + xim)*3+0] = value[0]
        im.data[(yim*im.width + xim)*3+1] = value[1]
        im.data[(yim*im.width + xim)*3+2] = value[2]
        i+=1

    y = 1
    for x in range(0, 7):
        xim = 0+x
        yim = im.height-1-(2+y)
        value = convertToRGB(int(data.palm_tip[i]/2))
        im.data[(yim*im.width + xim)*3+0] = value[0]
        im.data[(yim*im.width + xim)*3+1] = value[1]
        im.data[(yim*im.width + xim)*3+2] = value[2]
        i+=1

    y = 2
    for x in range(0, 7):
        xim = 0+x
        yim = im.height-1-(2+y)
        value = convertToRGB(int(data.palm_tip[i]/2))
        im.data[(yim*im.width + xim)*3+0] = value[0]
        im.data[(yim*im.width + xim)*3+1] = value[1]
        im.data[(yim*im.width + xim)*3+2] = value[2]
        i+=1

    y = 3
    for x in range(0, 5):
        xim = 1+x
        yim = im.height-1-(2+y)
        value = convertToRGB(int(data.palm_tip[i]/2))
        im.data[(yim*im.width + xim)*3+0] = value[0]
        im.data[(yim*im.width + xim)*3+1] = value[1]
        im.data[(yim*im.width + xim)*3+2] = value[2]
        i+=1

#    for i in range(0, 24):
#        value = convertToRGB(int(data.finger1_tip[i]/2))
#        im.data[i*3] = value[0]
#        im.data[i*3+1] = value[1]
#        im.data[i*3+2] = value[2]
    tactileImagepub.publish(im)

if __name__ == "__main__":
    print "Requesting pressure sensors info"
    pressure_info = get_pressure_sensors_info_client()
    print "sensor[0].center[0]: %s, %s, %s"%(pressure_info.sensor[0].center[0].x, pressure_info.sensor[0].center[0].y, pressure_info.sensor[0].center[0].z)

    pub = rospy.Publisher('/barrett_hand/tactile_markers', MarkerArray)
    tactileImagepub = rospy.Publisher('/barrett_hand/tactile_image', Image)

    rospy.init_node('hand_markers', anonymous=True)
    tf_listener = tf.TransformListener()
    rospy.Subscriber("/barrett_hand_controller/BHPressureState", BHPressureState, callback)
    rospy.spin()


