#!/usr/bin/env python
import roslib; roslib.load_manifest('barrett_hand_controller')

import sys
import rospy
import math
import copy

import std_msgs.msg
from std_msgs.msg import ColorRGBA
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image

import tf
from tf import *
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import *

update_on = "demand"

# ********************** interactive markers ****************************

def move_hand_client(prefix, f1, f2, f3, spread):
    rospy.wait_for_service('/' + prefix + '_hand/move_hand')
    try:
        move_hand = rospy.ServiceProxy('/' + prefix + '_hand/move_hand', BHMoveHand)
        resp1 = move_hand(f1, f2, f3, spread, 0.7, 0.7, 0.7, 0.7, 1000, 1000, 1000, 1000)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def processFeedback(feedback):
    global update_on

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        if feedback.control_name == "button1_control":
            update_on = "demand"
        elif feedback.control_name == "button2_control":
            update_on = "mouse"

    global spread_val
    global f1_val
    global f2_val
    global f3_val
    global prefix

    if feedback.marker_name == "spread_marker":
        val = 2*math.atan2(feedback.pose.orientation.z, feedback.pose.orientation.w)
        if val<-math.pi:
            val = 2*math.pi + val
        if val<0:
            val = -val
        elif val>math.pi:
            val = 2*math.pi - val
        spread_val = val
    elif feedback.marker_name == "f1_marker":
        val = -2.0*math.atan2(feedback.pose.orientation.y, feedback.pose.orientation.w)
        if val<-math.pi*0.5:
            val = 2*math.pi + val
        elif val>math.pi*1.5:
            val = val - 2*math.pi
        f1_val = val
    elif feedback.marker_name == "f2_marker":
        val = -2.0*math.atan2(feedback.pose.orientation.y, feedback.pose.orientation.w)
        if val<-math.pi*0.5:
            val = 2*math.pi + val
        elif val>math.pi*1.5:
            val = val - 2*math.pi
        f2_val = val
    elif feedback.marker_name == "f3_marker":
        val = -2.0*math.atan2(feedback.pose.orientation.x, feedback.pose.orientation.w)
        if val<-math.pi*0.5:
            val = 2*math.pi + val
        elif val>math.pi*1.5:
            val = val - 2*math.pi
        f3_val = val

    if ( (update_on == "mouse" and feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP) or (feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and feedback.control_name == "button1_control") ):
        move_hand_client(prefix, f1_val, f2_val, f3_val, spread_val)

def createSphereMarkerControl(scale, position, color):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale = scale
    marker.pose.position = position
    marker.color = color
    control = InteractiveMarkerControl()
    control.always_visible = True;
    control.markers.append( marker );
    return control

def createBoxMarkerControl(scale, position):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale = scale
    marker.pose.position = position
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    control = InteractiveMarkerControl()
    control.always_visible = True;
    control.markers.append( marker );
    return control

def run_int():
    global spread_val
    global f1_val
    global f2_val
    global f3_val

    spread_val = 0.0
    f1_val = 0.0
    f2_val = 0.0
    f3_val = 0.0

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer('/'+prefix+'_markers')

    button1Control = createSphereMarkerControl(Point(0.03,0.03,0.03), Point(0.1,0.02,-0.18), ColorRGBA(1,0,0,1))
    button1Control.interaction_mode = InteractiveMarkerControl.BUTTON
    button1Control.description="Update on demand"
    button1Control.name = "button1_control"
    button1Control.orientation_mode = InteractiveMarkerControl.VIEW_FACING

    button2Control = createSphereMarkerControl(Point(0.02,0.02,0.02), Point(0.1,-0.02,-0.18), ColorRGBA(0,0,1,1))
    button2Control.interaction_mode = InteractiveMarkerControl.BUTTON
    button2Control.description="Update on mouse-up"
    button2Control.name = "button2_control"
    button2Control.orientation_mode = InteractiveMarkerControl.VIEW_FACING

    menu_marker = InteractiveMarker()
    menu_marker.header.frame_id = prefix+"_HandPalmLink";
    menu_marker.name = "menu_marker"
    menu_marker.scale = 0.25
    menu_marker.controls.append(copy.deepcopy(button1Control))
    menu_marker.controls.append(copy.deepcopy(button2Control))
    server.insert(menu_marker, processFeedback);

    # spread
    int_spread_marker = InteractiveMarker()
    int_spread_marker.header.frame_id = prefix+"_HandPalmLink";
    int_spread_marker.name = "spread_marker";
    int_spread_marker.scale = 0.2
    rotate_control_spread = InteractiveMarkerControl()
    rotate_control_spread.name = "spread";
    rotate_control_spread.orientation = Quaternion(0,1,0,1)
    rotate_control_spread.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_spread_marker.controls.append(rotate_control_spread);
    int_spread_marker.controls.append( createBoxMarkerControl(Point(0.01,0.2,0.01), Point(0.0, 0.1, 0.0) ) )
    server.insert(int_spread_marker, processFeedback);

    # finger 1
    int_f1_marker = InteractiveMarker()
    int_f1_marker.header.frame_id = prefix+"_HandFingerOneKnuckleOneLink";
    int_f1_marker.name = "f1_marker";
    int_f1_marker.scale = 0.07
    int_f1_marker.pose.position = Point(0.05, 0, 0.0214)
    rotate_control_f1 = InteractiveMarkerControl()
    rotate_control_f1.name = "f1";
    rotate_control_f1.orientation = Quaternion(0,0,1,1)
    rotate_control_f1.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_f1_marker.controls.append(rotate_control_f1);
    int_f1_marker.controls.append( createBoxMarkerControl(Point(0.1,0.005,0.005), Point(0.05, 0.0, 0.0) ) )
    server.insert(int_f1_marker, processFeedback);

    # finger 2
    int_f2_marker = InteractiveMarker()
    int_f2_marker.header.frame_id = prefix+"_HandFingerTwoKnuckleOneLink";
    int_f2_marker.name = "f2_marker";
    int_f2_marker.scale = 0.07
    int_f2_marker.pose.position = Point(0.05, 0, 0.0214)
    rotate_control_f2 = InteractiveMarkerControl()
    rotate_control_f2.name = "f2";
    rotate_control_f2.orientation = Quaternion(0,0,1,1)
    rotate_control_f2.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_f2_marker.controls.append(rotate_control_f2);
    int_f2_marker.controls.append( createBoxMarkerControl(Point(0.1,0.005,0.005), Point(0.05, 0.0, 0.0) ) )
    server.insert(int_f2_marker, processFeedback);

    # finger 3
    int_f3_marker = InteractiveMarker()
    int_f3_marker.header.frame_id = prefix+"_HandPalmLink";
    int_f3_marker.name = "f3_marker";
    int_f3_marker.scale = 0.07
    int_f3_marker.pose.position = Point(0, -0.05, 0.080197)
    rotate_control_f3 = InteractiveMarkerControl()
    rotate_control_f3.name = "f3";
    rotate_control_f3.orientation = Quaternion(1,0,0,1)
    rotate_control_f3.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_f3_marker.controls.append(rotate_control_f3);
    int_f3_marker.controls.append( createBoxMarkerControl(Point(0.005,0.1,0.005), Point(0.0, -0.05, 0.0) ) )
    server.insert(int_f3_marker, processFeedback);

    server.applyChanges();

# ****************** markers **********************

def get_pressure_sensors_info_client():
    global prefix
    rospy.wait_for_service('/' + prefix + '_hand/get_pressure_info')
    try:
        get_pressure_sensors_info = rospy.ServiceProxy('/' + prefix + '_hand/get_pressure_info', BHGetPressureInfo)
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
    global tf_listener
    global prefix
    global pressure_info
    global pub
    global tactileImagepub
    m = MarkerArray()

    frame_id = []
    frame_id.append(prefix+"_HandFingerOneKnuckleThreeLink")
    frame_id.append(prefix+"_HandFingerTwoKnuckleThreeLink")
    frame_id.append(prefix+"_HandFingerThreeKnuckleThreeLink")
    frame_id.append(prefix+"_HandPalmLink")


    tran = []
    rotat = []
    (transl, rot) = tf_listener.lookupTransform(prefix+'_HandPalmLink', frame_id[0], rospy.Time(0))
    tran.append(transl)
    rotat.append(rot)
    (transl, rot) = tf_listener.lookupTransform(prefix+'_HandPalmLink', frame_id[1], rospy.Time(0))
    tran.append(transl)
    rotat.append(rot)
    (transl, rot) = tf_listener.lookupTransform(prefix+'_HandPalmLink', frame_id[2], rospy.Time(0))
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

    tactileImagepub.publish(im)

def run_mark():
    global tf_listener
    global pressure_info
    global pub
    global tactileImagepub
    print "Requesting pressure sensors info"
    pressure_info = get_pressure_sensors_info_client()
    print "sensor[0].center[0]: %s, %s, %s"%(pressure_info.sensor[0].center[0].x, pressure_info.sensor[0].center[0].y, pressure_info.sensor[0].center[0].z)

    pub = rospy.Publisher('/' + prefix + '_hand/tactile_markers', MarkerArray)
    tactileImagepub = rospy.Publisher('/' + prefix + '_hand/tactile_image', Image)

    tf_listener = tf.TransformListener()
    rospy.Subscriber('/' + prefix + '_hand/BHPressureState', BHPressureState, callback)


if __name__ == '__main__':
    a = []
    for arg in sys.argv:
        a.append(arg)

    if len(a) > 1:
        prefix = a[1]
    else:
        print "Usage: %s prefix [noint] [notac]"%a[0]
        exit(0)

    noint = 0
    notac = 0
    if len(a) > 2:
        if a[2] == 'noint':
            noint = 1
        if a[2] == 'notac':
            notac = 1

    if len(a) > 3:
        if a[3] == 'noint':
            noint = 1
        if a[3] == 'notac':
            notac = 1

    if noint == 1 and notac == 1:
        exit(0)

    rospy.init_node(prefix+'_hand_markers', anonymous=True)

    if noint == 0:
        print "int"
        run_int()
    if notac == 0:
        print "tac"
        run_mark()

    # start the ROS main loop
    rospy.spin()


