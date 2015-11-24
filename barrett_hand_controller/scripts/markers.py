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
import copy

import std_msgs.msg
from std_msgs.msg import ColorRGBA
from barrett_hand_controller_msgs.msg import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
import PyKDL
import tf_conversions.posemath as pm

import barrett_hand_interface

import tf
from tf import *
from tf2_msgs.msg import *

class BarrettHandMarkers:

    # ********************** interactive markers ****************************

    def processFeedback(self, feedback):

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            if feedback.control_name == "button1_control":
                self.update_on = "demand"
            elif feedback.control_name == "button2_control":
                self.update_on = "mouse"


        if feedback.marker_name == "spread_marker":
            val = 2*math.atan2(feedback.pose.orientation.z, feedback.pose.orientation.w)
            if val<-math.pi:
                val = 2*math.pi + val
            if val<0:
                val = -val
            elif val>math.pi:
                val = 2*math.pi - val
            self.spread_val = val
        elif feedback.marker_name == "f3_marker":
            val = -2.0*math.atan2(feedback.pose.orientation.x, feedback.pose.orientation.w)
            if val<-math.pi*0.5:
                val = 2*math.pi + val
            elif val>math.pi*1.5:
                val = val - 2*math.pi
            self.f3_val = val
        else:
            val = -2.0*math.atan2(feedback.pose.orientation.y, feedback.pose.orientation.w)
            if val<-math.pi*0.5:
                val = 2*math.pi + val
            elif val>math.pi*1.5:
                val = val - 2*math.pi

        if feedback.marker_name == "f1_marker":
            self.f1_val = val
        elif feedback.marker_name == "f2_marker":
            self.f2_val = val

        if ( (self.update_on == "mouse" and feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP) or (feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and feedback.control_name == "button1_control") ):
            self.bh.moveHand([self.f1_val, self.f2_val, self.f3_val, self.spread_val], [self.velocity, self.velocity, self.velocity, self.velocity], [4000, 4000, 4000, 4000], self.stop_force, hold=self.spread_hold)

    def createSphereMarkerControl(self, scale, position, color):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale = scale
        marker.pose.position = position
        marker.color = color
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def createBoxMarkerControl(self, scale, position):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale = scale
        marker.pose.position = position
        marker.color = ColorRGBA(0.5,0.5,0.5,1)
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append( marker );
        return control

    def stopForceCb(self, feedback):
        for force in self.force_menu_id_map:
            if self.force_menu_id_map[force] == feedback.menu_entry_id:
                print "STOP force: ", force
                self.stop_force = force;
                break

    def spreadHoldCb(self, feedback):
        if self.menu_spread_enable_id == feedback.menu_entry_id:
            print "spread HOLD enabled"
            self.spread_hold = True
        elif self.menu_spread_disable_id == feedback.menu_entry_id:
            print "spread HOLD disabled"
            self.spread_hold = False

    def velocityCb(self, feedback):
        for velocity in self.velocity_menu_id_map:
            if self.velocity_menu_id_map[velocity] == feedback.menu_entry_id:
                print "velocity: ", velocity
                self.velocity = velocity
                break

    def run_int(self):
        self.spread_val = 0.0
        self.f1_val = 0.0
        self.f2_val = 0.0
        self.f3_val = 0.0

        self.menu_handler = MenuHandler()

        self.menu_stop_force = self.menu_handler.insert( "STOP force" )
#        stop_forces_list = [30, 50, 70, 100, 150, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 4000]
        stop_forces_list = [0.125, 0.25, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0, 2.8, 4.0]
        self.force_menu_id_map = {}
        for force in stop_forces_list:
            self.force_menu_id_map[force] = self.menu_handler.insert( str(force), parent=self.menu_stop_force, callback=self.stopForceCb )

        self.menu_spread_hold = self.menu_handler.insert( "spread HOLD" )
        self.menu_spread_enable_id = self.menu_handler.insert( "enable", parent=self.menu_spread_hold, callback=self.spreadHoldCb )
        self.menu_spread_disable_id = self.menu_handler.insert( "disable", parent=self.menu_spread_hold, callback=self.spreadHoldCb )

        self.menu_velocity = self.menu_handler.insert( "velocity" )
        velocity_list = [0.3, 0.5, 0.7, 1.0, 1.2]
        self.velocity_menu_id_map = {}
        for velocity in velocity_list:
            self.velocity_menu_id_map[velocity] = self.menu_handler.insert( str(velocity), parent=self.menu_velocity, callback=self.velocityCb )

        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer('/'+self.prefix+'_markers')

        button1Control = self.createSphereMarkerControl(Point(0.03,0.03,0.03), Point(0.1,0.02,-0.18), ColorRGBA(1,0,0,1))
        button1Control.interaction_mode = InteractiveMarkerControl.BUTTON
        button1Control.description="Update on demand"
        button1Control.name = "button1_control"
        button1Control.orientation_mode = InteractiveMarkerControl.VIEW_FACING

        button2Control = self.createSphereMarkerControl(Point(0.02,0.02,0.02), Point(0.1,-0.02,-0.18), ColorRGBA(0,0,1,1))
        button2Control.interaction_mode = InteractiveMarkerControl.BUTTON
        button2Control.description="Update on mouse-up"
        button2Control.name = "button2_control"
        button2Control.orientation_mode = InteractiveMarkerControl.VIEW_FACING

        menu_marker = InteractiveMarker()
        menu_marker.header.frame_id = self.prefix+"_HandPalmLink";
        menu_marker.name = "menu_marker"
        menu_marker.scale = 0.25
        menu_marker.controls.append(copy.deepcopy(button1Control))
        menu_marker.controls.append(copy.deepcopy(button2Control))
        self.server.insert(menu_marker, self.processFeedback);

        # spread
        int_spread_marker = InteractiveMarker()
        int_spread_marker.header.frame_id = self.prefix+"_HandPalmLink";
        int_spread_marker.name = "spread_marker";
        int_spread_marker.scale = 0.2
        rotate_control_spread = InteractiveMarkerControl()
        rotate_control_spread.name = "spread";
        rotate_control_spread.orientation = Quaternion(0,1,0,1)
        rotate_control_spread.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_spread_marker.controls.append(rotate_control_spread);
        int_spread_marker.controls.append( self.createBoxMarkerControl(Point(0.01,0.2,0.01), Point(0.0, 0.1, 0.0) ) )
        self.server.insert(int_spread_marker, self.processFeedback);

        # finger 1
        int_f1_marker = InteractiveMarker()
        int_f1_marker.header.frame_id = self.prefix+"_HandFingerOneKnuckleOneLink";
        int_f1_marker.name = "f1_marker";
        int_f1_marker.scale = 0.07
        int_f1_marker.pose.position = Point(0.05, 0, 0.0214)
        rotate_control_f1 = InteractiveMarkerControl()
        rotate_control_f1.name = "f1";
        rotate_control_f1.orientation = Quaternion(0,0,1,1)
        rotate_control_f1.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_f1_marker.controls.append(rotate_control_f1);
        int_f1_marker.controls.append( self.createBoxMarkerControl(Point(0.1,0.005,0.005), Point(0.05, 0.0, 0.0) ) )
        self.server.insert(int_f1_marker, self.processFeedback);

        # finger 2
        int_f2_marker = InteractiveMarker()
        int_f2_marker.header.frame_id = self.prefix+"_HandFingerTwoKnuckleOneLink";
        int_f2_marker.name = "f2_marker";
        int_f2_marker.scale = 0.07
        int_f2_marker.pose.position = Point(0.05, 0, 0.0214)
        rotate_control_f2 = InteractiveMarkerControl()
        rotate_control_f2.name = "f2";
        rotate_control_f2.orientation = Quaternion(0,0,1,1)
        rotate_control_f2.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_f2_marker.controls.append(rotate_control_f2);
        int_f2_marker.controls.append( self.createBoxMarkerControl(Point(0.1,0.005,0.005), Point(0.05, 0.0, 0.0) ) )
        self.server.insert(int_f2_marker, self.processFeedback);

        # finger 3
        int_f3_marker = InteractiveMarker()
        int_f3_marker.header.frame_id = self.prefix+"_HandPalmLink";
        int_f3_marker.name = "f3_marker";
        int_f3_marker.scale = 0.07
        int_f3_marker.pose.position = Point(0, -0.05, 0.080197)
        rotate_control_f3 = InteractiveMarkerControl()
        rotate_control_f3.name = "f3";
        rotate_control_f3.orientation = Quaternion(1,0,0,1)
        rotate_control_f3.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_f3_marker.controls.append(rotate_control_f3);
        int_f3_marker.controls.append( self.createBoxMarkerControl(Point(0.005,0.1,0.005), Point(0.0, -0.05, 0.0) ) )
        self.server.insert(int_f3_marker, self.processFeedback);

        self.menu_handler.apply( self.server, "menu_marker" )

        self.server.applyChanges();

    def __init__(self, prefix):
        self.prefix = prefix
        self.bh = barrett_hand_interface.BarrettHand(self.prefix)
        self.stop_force = 30
        self.spread_hold = True
        self.velocity = 1.0
        self.update_on = "demand"
        self.run_int()

        self.pub0 = rospy.Publisher('optoforce0_pos', Vector3Stamped, queue_size=10)
        self.pub1 = rospy.Publisher('optoforce1_pos', Vector3Stamped, queue_size=10)
        self.pub2 = rospy.Publisher('optoforce2_pos', Vector3Stamped, queue_size=10)
        self.listener = tf.TransformListener()

    def spin(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                (trans0,rot0) = self.listener.lookupTransform('/right_HandPalmLink', '/right_HandFingerOneKnuckleThreeOptoforce', rospy.Time(0))
                (trans1,rot1) = self.listener.lookupTransform('/right_HandPalmLink', '/right_HandFingerTwoKnuckleThreeOptoforce', rospy.Time(0))
                (trans2,rot2) = self.listener.lookupTransform('/right_HandPalmLink', '/right_HandFingerThreeKnuckleThreeOptoforce', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            v0 = Vector3Stamped()
            v1 = Vector3Stamped()
            v2 = Vector3Stamped()
            v0.header.stamp = rospy.Time.now()
            v1.header.stamp = rospy.Time.now()
            v2.header.stamp = rospy.Time.now()
            v0.vector.x = trans0[0]
            v0.vector.y = trans0[1]
            v0.vector.z = trans0[2]
            v1.vector.x = trans1[0]
            v1.vector.y = trans1[1]
            v1.vector.z = trans1[2]
            v2.vector.x = trans2[0]
            v2.vector.y = trans2[1]
            v2.vector.z = trans2[2]
            self.pub0.publish(v0)
            self.pub1.publish(v1)
            self.pub2.publish(v2)
        


if __name__ == '__main__':
    a = []
    for arg in sys.argv:
        a.append(arg)

    if len(a) > 1:
        prefix = a[1]
    else:
        print "Usage: %s prefix"%a[0]
        exit(0)

    rospy.init_node(prefix+'_hand_markers', anonymous=True)

    bhm = BarrettHandMarkers(prefix)

    # start the ROS main loop
    bhm.spin()


