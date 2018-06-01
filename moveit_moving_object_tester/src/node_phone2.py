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

def callback2(msg):
    elbowx = msg.x
    elbowy = msg.y
    elbowz = msg.z
    global elbowx, elbowy, elbowz

def callback(msg):
    pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=10)
  
    moveit_commander.roscpp_initialize(sys.argv)

    scene = moveit_commander.PlanningSceneInterface()

    robot = moveit_commander.RobotCommander()

    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id  = 'world'
    collision_object.id = 'test_object2'
    collision_object.operation = collision_object.ADD

    shape = shape_msgs.msg.SolidPrimitive()
    shape.type = shape.CYLINDER
    shape.dimensions = [0.40, 0.075]

    shape_pose = geometry_msgs.msg.Pose()


    shape_pose.position.x = elbowx + ( cos(radians(msg.twist.angular.z+90)) * sin(radians(msg.twist.angular.x+90)) * (0.5*shape.dimensions[0]-0.05))
    shape_pose.position.y = elbowy + ( sin(radians(msg.twist.angular.z+90)) * sin(radians(msg.twist.angular.x+90)) * (0.5*shape.dimensions[0]-0.05))
    shape_pose.position.z = elbowz + (-cos(radians(msg.twist.angular.x+90)) * (0.5*shape.dimensions[0]-0.05))

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
    rospy.Subscriber('elbow_position', Point, callback2)
    rospy.Subscriber('phone2', TwistStamped, callback)
    rospy.spin()

if __name__ == '__main__':
  try:
    listener()
  except rospy.ROSInterruptException:
    pass
