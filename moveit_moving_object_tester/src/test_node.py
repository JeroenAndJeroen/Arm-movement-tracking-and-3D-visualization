#!/usr/bin/env python

import sys
#import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import shape_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

global xorient, yorient, zorient

xorient = 0
yorient = 0
zorient = 0

def callback(msg):
    xorient = msg.angular.x
    yorient = msg.angular.y
    zorient = msg.angular.z


def object_publisher():
  pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=10)
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.subscriber("phone1", Twist, callback)

  rospy.init_node('moveit_test_node', anonymous=True)
  rate = rospy.Rate(30)

  scene = moveit_commander.PlanningSceneInterface()

  robot = moveit_commander.RobotCommander()

  #ground_pose = geometry_msgs.msg.PoseStamped()

  #ground_pose.header.frame_id = robot.get_planning_frame()
  #ground_pose.pose.position.x = 0.0
  #ground_pose.pose.position.y = 0.0
  #ground_pose.pose.position.z = 0.0
  #ground_pose.pose.orientation.w = 1.0
  #scene.add_plane('ground', ground_pose, (0,0,1), 0)

  

  y_pos = 0.0
  y_itt = 0.02

  

  while not rospy.is_shutdown():
    
    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id  = 'world'
    collision_object.id = 'test_object'
    collision_object.operation = collision_object.ADD

    shape = shape_msgs.msg.SolidPrimitive()
    shape.type = shape.CYLINDER
    shape.dimensions = [0.3, 0.05]


    y_pos += y_itt

    if y_pos > 1.0 or y_pos < -1.0:
      y_itt = -y_itt

    shape_pose = geometry_msgs.msg.Pose()
    #shape_pose = geometry_msgs.msg.PoseStamped()
    shape_pose.position.x = 0.5
    shape_pose.position.z = 0.5
    shape_pose.position.y = y_pos
    shape_pose.orientation.x = xorient
    shape_pose.orientation.y = yorient
    shape_pose.orientation.z = zorient
    shape_pose.orientation.w = 1.0

    #shape_pose.header.frame_id = robot.get_planning_frame()

    collision_object.primitives = [shape]
    collision_object.primitive_poses = [shape_pose]
      
    pub.publish(collision_object)  

    #scene.add_box('box', shape_pose, (0.5, 0.5, 0.5))
    rate.sleep()
 


if __name__ == '__main__':
  try:
    object_publisher()
  except rospy.ROSInterruptException:
    pass



'''
def move_group_python_interface_add_object():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  print "============ Robot Groups:"
  print robot.get_group_names()

  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  scene.add_box('test_box', )

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
'''
