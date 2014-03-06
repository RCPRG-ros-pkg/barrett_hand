#!/usr/bin/env python
import roslib; roslib.load_manifest('barrett_hand_controller')

import sys
import rospy
import math
import copy

from std_msgs.msg import ColorRGBA
from barrett_hand_controller.msg import *
from barrett_hand_controller_srvs.srv import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

prefix = "right"
update_on = "demand"

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
        goal = BHMoveActionGoal()
        goal.header.frame_id = prefix+"_HandPalmLink"
        goal.header.stamp = rospy.Time.now()
        goal.goal.finger[0] = f1_val
        goal.goal.finger[1] = f2_val
        goal.goal.finger[2] = f3_val
        goal.goal.spread = spread_val
        goal.goal.fingerVel = 1.0
        goal.goal.spreadVel = 2.0
        global spread_pub
        spread_pub.publish(goal)

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

if __name__ == "__main__":
    rospy.init_node('int_hand_markers', anonymous=True)

    spread_val = 0.0
    f1_val = 0.0
    f2_val = 0.0
    f3_val = 0.0

    spread_pub = rospy.Publisher('/move_hand/goal', BHMoveActionGoal)

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("int_hand_markers")

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

    # start the ROS main loop
    rospy.spin()


