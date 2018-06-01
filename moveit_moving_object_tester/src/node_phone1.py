#!/usr/bin/env python

import sys
#import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import shape_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from math import radians
from math import sin
from math import cos
from time import sleep
'''
def callbackCorrection(msg):
    print 'Wait 3 seconds and keep the phones pointed in the same direction'
    for second in range(1,4):
        if second == 3:
            zCorrectionPhone1 = msg.twist.angular.z
        sleep(1)
        print second, ' second(s)'
    print zCorrectionPhone1
    global zCorrectionPhone1

rospy.Subscriber("phone1", TwistStamped, callbackCorrection)
'''
def callback(msg):

    
    pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=10)
    elbow_pub = rospy.Publisher('/elbow_position', Point, queue_size=10)
    
    moveit_commander.roscpp_initialize(sys.argv)

    scene = moveit_commander.PlanningSceneInterface()

    robot = moveit_commander.RobotCommander()

    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id  = 'world'
    collision_object.id = 'test_object'
    collision_object.operation = collision_object.ADD

    shape = shape_msgs.msg.SolidPrimitive()
    shape.type = shape.CYLINDER
    shape.dimensions = [0.4, 0.075]

    shape_pose = geometry_msgs.msg.Pose()
    shape_pose.position.x = 0.5 + (cos(radians(msg.twist.angular.z+90)) * sin(radians(msg.twist.angular.x+90)) * 0.5*shape.dimensions[0])
    shape_pose.position.y = 0.5 + (sin(radians(msg.twist.angular.z+90))  * sin(radians(msg.twist.angular.x+90)) * 0.5*shape.dimensions[0])
    shape_pose.position.z = 0.5 + (-cos(radians(msg.twist.angular.x+90)) * 0.5*shape.dimensions[0])
    
    elbowposx = 0.5 + (cos(radians(msg.twist.angular.z+90)) * sin(radians(msg.twist.angular.x+90)) * shape.dimensions[0])
    elbowposy = 0.5 + (sin(radians(msg.twist.angular.z+90))  * sin(radians(msg.twist.angular.x+90)) * shape.dimensions[0])
    elbowposz = 0.5 + (-cos(radians(msg.twist.angular.x+90)) * shape.dimensions[0])

    elbow_pub.publish(elbowposx, elbowposy, elbowposz)

    q = quaternion_from_euler(radians(msg.twist.angular.x+90), 0, radians(msg.twist.angular.z)) 
    
    shape_pose.orientation.x = q[0]
    shape_pose.orientation.y = q[1]
    shape_pose.orientation.z = q[2]
    shape_pose.orientation.w = q[3]

    collision_object.primitives = [shape]
    collision_object.primitive_poses = [shape_pose]
      
    pub.publish(collision_object)  

    print msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
    print shape_pose.position.x, shape_pose.position.y, shape_pose.position.z
    print 
    
def listener():
    rospy.init_node('moveit_test_node', anonymous=True)
    rospy.Subscriber("phone1", TwistStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
