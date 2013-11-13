#!/usr/bin/env python
import roslib; roslib.load_manifest('barrett_hand_controller')

import sys
import rospy
import math

import std_msgs.msg
from barrett_hand_controller.msg import *
from barrett_hand_controller.srv import *
from graspit_ros_planning_msgs.srv import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Transform
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

def callback(data):
    global pressure_info
    global tf_listener
    m = MarkerArray()

    frame_id = []
    frame_id.append("right_HandFingerOneKnuckleThreeLink")
    frame_id.append("right_HandFingerTwoKnuckleThreeLink")
    frame_id.append("right_HandFingerThreeKnuckleThreeLink")
    frame_id.append("right_HandPalmLink")

    contacts = []
    forces = []
    transforms = []
    counter = 0
    for sens in range(0, 4):
        print "sensor: %s"%(sens)
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

#            tran = tf_listener.lookupTransform('right_HandPalmLink', frame_id[sens], rospy.Time(0))
#            print "ret: %s"%(ret)
	    (trans,rot) = tf_listener.lookupTransform('right_HandPalmLink', frame_id[sens], rospy.Time(0))
#            print "trans: ",trans
#            print "rot: ",rot
            tran = Transform()
            tran.translation.x = trans[0]
            tran.translation.y = trans[1]
            tran.translation.z = trans[2]

            tran.rotation.x = rot[0]
            tran.rotation.y = rot[1]
            tran.rotation.z = rot[2]
            tran.rotation.w = rot[3]

            px = pressure_info.sensor[sens].center[i].x
            py = pressure_info.sensor[sens].center[i].y
            pz = pressure_info.sensor[sens].center[i].z
            cx = cx/length*scale*0.01
            cy = cy/length*scale*0.01
            cz = cz/length*scale*0.01
            print "  force: %s"%(scale)
            if scale > 0.5:
                contacts.append( Vector3(px, py, pz) )
                forces.append( Vector3(cx,cy,cz) )
                transforms.append(tran)
                counter += 1

    print "got it: %s"%(counter)

#tf2_msgs/LookupTransform.action

    # Creates the SimpleActionClient, passing the type of the action
#    client = actionlib.SimpleActionClient('/move_hand', tf2_msgs.msg.LookupTransformAction)

    # Waits until the action server has started up and started
    # listening for goals.
#    client.wait_for_server()

    # Creates a goal to send to the action server.
#    goal = tf2_msgs.msg.LookupTransformGoal()
    
#    goal.fingerVel = 0.2
#    goal.spreadVel = 0.2
#    goal.spread = 3.14
#    goal.finger[0] = 1.2
#    goal.finger[1] = 1.2
#    goal.finger[2] = 1.2
    
    # Sends the goal to the action server.
#    client.send_goal(goal)
#    client.wait_for_result()


    get_quality = rospy.ServiceProxy('/ros_graspit_interface/get_quality', GetQuality)
    resp1 = get_quality(contacts, forces, transforms)

    rospy.spin()

#def tf_callback(data):


if __name__ == "__main__":
    print "Requesting pressure sensors info"
    pressure_info = get_pressure_sensors_info_client()
    rospy.init_node('quality_measure', anonymous=True)
    rospy.Subscriber("/barrett_hand_controller/BHPressureState", BHPressureState, callback)
    tf_listener = tf.TransformListener()
    rospy.spin()

