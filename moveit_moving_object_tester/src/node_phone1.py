#!/usr/bin/env python

# Import packages
import sys
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

# Variable to get the first z_orientation to make a correction
FirstDataPoint = True

# Callback function for Collision Object
def callback(msg):

    # Make FirstDataPoint a global variable
    global FirstDataPoint

    # Check if first data point and get z_orientation to make correction
    if FirstDataPoint:
        correction = msg.twist.angular.z
	global correction
        FirstDataPoint = False
	print correction

    # Create publishers to publish collision object to RViz and send give start point for underarm
    pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=10)
    elbow_pub = rospy.Publisher('/elbow_position', Point, queue_size=10)
    
    # MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    # Create object
    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id  = 'world'
    collision_object.id = 'test_object'
    collision_object.operation = collision_object.ADD

    # Define object shape
    shape = shape_msgs.msg.SolidPrimitive()
    shape.type = shape.CYLINDER
    shape.dimensions = [0.4, 0.075]

    # Define object position
    shape_pose = geometry_msgs.msg.Pose()
    shape_pose.position.x = 0.5 + (cos(radians(msg.twist.angular.z-correction+90)) * sin(radians(msg.twist.angular.x+90)) * 0.5*shape.dimensions[0])
    shape_pose.position.y = 0.5 + (sin(radians(msg.twist.angular.z-correction+90))  * sin(radians(msg.twist.angular.x+90)) * 0.5*shape.dimensions[0])
    shape_pose.position.z = 0.5 + (-cos(radians(msg.twist.angular.x+90)) * 0.5*shape.dimensions[0])
    
    # Get end position of cylinder (elbow position
    elbowposx = 0.5 + (cos(radians(msg.twist.angular.z-correction+90)) * sin(radians(msg.twist.angular.x+90)) * shape.dimensions[0])
    elbowposy = 0.5 + (sin(radians(msg.twist.angular.z-correction+90))  * sin(radians(msg.twist.angular.x+90)) * shape.dimensions[0])
    elbowposz = 0.5 + (-cos(radians(msg.twist.angular.x+90)) * shape.dimensions[0])

    # Publish elbow position
    elbow_pub.publish(elbowposx, elbowposy, elbowposz)

    # Turn orientation data in to quaternions
    q = quaternion_from_euler(radians(msg.twist.angular.x+90), 0, radians(msg.twist.angular.z-correction)) 
    
    # Define object orientation
    shape_pose.orientation.x = q[0]
    shape_pose.orientation.y = q[1]
    shape_pose.orientation.z = q[2]
    shape_pose.orientation.w = q[3]

    #assign object properties to object
    collision_object.primitives = [shape]
    collision_object.primitive_poses = [shape_pose]
      
    # Publish collision object
    pub.publish(collision_object)  

    #print phone orientation in degrees, object position and correction to check if correct
    print msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
    print shape_pose.position.x, shape_pose.position.y, shape_pose.position.z
    print correction
    print
    
# Listener function to define subscriber and keep node running
def listener():
    rospy.init_node('moveit_test_node', anonymous=True)
    rospy.Subscriber("phone1", TwistStamped, callback)
    rospy.spin()

# Run function until ROS is interrupted
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
