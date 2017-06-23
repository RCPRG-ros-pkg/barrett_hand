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

import roslib; roslib.load_manifest('barrett_hand_tactile')

import sys
import rospy
import math

import std_msgs.msg
from std_msgs.msg import ColorRGBA
from barrett_hand_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
import PyKDL
import tf_conversions.posemath as pm

import barrett_hand_tactile_interface

class BarrettHandTactileMarkers:

    # ****************** markers **********************

    def convertToRGB(self, value):
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

    def spin(self):
        
        while not rospy.is_shutdown():

            m = MarkerArray()

            stamp, f1_tact, f2_tact, f3_tact, palm_tact = self.bh.getTactileData()
            finger_skin_data = []
            finger_skin_data.append(f1_tact)
            finger_skin_data.append(f2_tact)
            finger_skin_data.append(f3_tact)
            finger_skin_data.append(palm_tact)

            frame_id = []
            frame_id.append(self.prefix+"_HandFingerOneKnuckleThreeLink")
            frame_id.append(self.prefix+"_HandFingerTwoKnuckleThreeLink")
            frame_id.append(self.prefix+"_HandFingerThreeKnuckleThreeLink")
            frame_id.append(self.prefix+"_HandPalmLink")

            arrows = False

            if arrows:
                for sens in range(0, 4):
                    for i in range(0, 24):
                        halfside1 = PyKDL.Vector(pressure_info.sensor[sens].halfside1[i].x, pressure_info.sensor[sens].halfside1[i].y, pressure_info.sensor[sens].halfside1[i].z)
                        halfside2 = PyKDL.Vector(pressure_info.sensor[sens].halfside2[i].x, pressure_info.sensor[sens].halfside2[i].y, pressure_info.sensor[sens].halfside2[i].z)
                        # calculate cross product: halfside1 x halfside2
                        norm = halfside1*halfside2
                        norm.Normalize()
                        scale = 0
                        if sens == 0:
                            scale = data.finger1_tip[i]/256.0
                        elif sens == 1:
                            scale = data.finger2_tip[i]/256.0
                        elif sens == 2:
                            scale = data.finger3_tip[i]/256.0
                        else:
                            scale = data.palm_tip[i]/256.0
                        norm = norm*scale*0.01
                        marker = Marker()
                        marker.header.frame_id = frame_id[sens]
                        marker.header.stamp = rospy.Time.now()
                        marker.ns = frame_id[sens]
                        marker.id = i
                        marker.type = 0
                        marker.action = 0
                        marker.points.append(Point(pressure_info.sensor[sens].center[i].x,pressure_info.sensor[sens].center[i].y,pressure_info.sensor[sens].center[i].z))
                        marker.points.append(Point(pressure_info.sensor[sens].center[i].x+norm.x(), pressure_info.sensor[sens].center[i].y+norm.y(), pressure_info.sensor[sens].center[i].z+norm.z()))
                        marker.pose = Pose( Point(0,0,0), Quaternion(0,0,0,1) )
                        marker.scale = Vector3(0.001, 0.002, 0)
                        marker.color = ColorRGBA(1,0,0,1)
                        m.markers.append(marker)
                self.pub.publish(m)
            else:
                for sens in range(0, 4):
                    for i in range(0, 24):
                        value = self.convertToRGB(int(finger_skin_data[sens][i]/2))

                        marker = Marker()
                        marker.header.frame_id = frame_id[sens]
                        marker.header.stamp = rospy.Time.now()
                        marker.ns = frame_id[sens]
                        marker.id = i
                        marker.type = 1
                        marker.action = 0
                        if sens < 3:
                            marker.pose = pm.toMsg(self.bh.pressure_frames[i])
                            marker.scale = Vector3(self.bh.pressure_cells_size[i][0], self.bh.pressure_cells_size[i][1], 0.004)
                        else:
                            marker.pose = pm.toMsg(self.bh.palm_pressure_frames[i])
                            marker.scale = Vector3(self.bh.palm_pressure_cells_size[i][0], self.bh.palm_pressure_cells_size[i][1], 0.004)
                        marker.color = ColorRGBA(1.0/255.0*value[0], 1.0/255.0*value[1], 1.0/255.0*value[2],1)
                        m.markers.append(marker)
                self.pub.publish(m)


        #  palm   f1  f2  f3
        #         xxx xxx xxx
        #         xxx xxx xxx 
        #  xxxxx  xxx xxx xxx
        # xxxxxxx xxx xxx xxx
        # 56789xx 9xx 9xx 9xx
        #  01234  678 678 678
        #         345 345 345
        #         012 012 012

            im = Image()
            im.height = 8
            im.width = 19
            im.encoding = "rgb8"
            im.is_bigendian = 0
            im.step = im.width*3
            im.data = [0]*(im.step*im.height)
            for finger in range(0, 3):
                for y in range(0, 8):
                    for x in range(0, 3):
                        xim = 8 + x + finger * 4
                        yim = im.height-1-y
                        value = self.convertToRGB(int(finger_skin_data[finger][y*3+x]/2))
                        im.data[(yim*im.width + xim)*3+0] = value[0]
                        im.data[(yim*im.width + xim)*3+1] = value[1]
                        im.data[(yim*im.width + xim)*3+2] = value[2]

            palm_im_x = [1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5]
            palm_im_y = [5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2]

            for i in range(0, 24):
                xim = palm_im_x[i]
                yim = palm_im_y[i]
                value = self.convertToRGB(int(finger_skin_data[3][i]/2))
                im.data[(yim*im.width + xim)*3+0] = value[0]
                im.data[(yim*im.width + xim)*3+1] = value[1]
                im.data[(yim*im.width + xim)*3+2] = value[2]

            self.tactileImagepub.publish(im)
            rospy.sleep(0.1)

    def run_mark(self):
        self.pub = rospy.Publisher('/' + self.prefix + '_hand/tactile_markers', MarkerArray, queue_size=100)
        self.tactileImagepub = rospy.Publisher('/' + self.prefix + '_hand/tactile_image', Image, queue_size=100)

    def __init__(self, prefix):
        self.prefix = prefix
        self.bh = barrett_hand_tactile_interface.BarrettHandTactileInterface(self.prefix)
        self.run_mark()
        rospy.sleep(1)

if __name__ == '__main__':
    a = []
    for arg in sys.argv:
        a.append(arg)

    if len(a) > 1:
        prefix = a[1]
    else:
        print "Usage: %s prefix"%a[0]
        exit(0)

    rospy.init_node(prefix+'_hand_tactile_vis', anonymous=True)
    bhm = BarrettHandTactileMarkers(prefix)
    bhm.spin()


