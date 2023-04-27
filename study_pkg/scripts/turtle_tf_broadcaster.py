#!/usr/bin/env python
import rospy
import tf
import math as math
import time
from tf.transformations import quaternion_from_euler
from turtlesim.msg import Pose

# Callback function
def handle_turtle_pose(msg):
    # Get broadcaster object
    angle=int((time.time() * 1000))%360
    x=1*math.cos(math.radians(angle))
    y=1*math.sin(math.radians(angle))
    br = tf.TransformBroadcaster()
    # Broadcast TF trasform (world -> turtlename)
    br.sendTransform((msg.x, msg.y, 0),
                     quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")
    br.sendTransform((x, y, 0),
                     quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     "carrot",
                     "turtle1")

# Subscribe to /input_pose topic - just gonna remap it to work with
if __name__=='__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle_tf_name')
    rospy.Subscriber('input_pose',
                 Pose,
                 handle_turtle_pose)
# You spin my head right round, right round - Florida =)
# Just handle all topic messages until node (or ROS) is working
rospy.spin()
