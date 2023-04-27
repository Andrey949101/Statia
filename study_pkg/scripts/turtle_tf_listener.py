#!/usr/bin/env python
import rospy

import math
import tf
from geometry_msgs.msg import Twist
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    msg = Twist()

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/turtle2', '/carrot', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        msg.linear.x = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        msg.angular.z = 4 * math.atan2(trans[1], trans[0])

        turtle_vel.publish(msg)

        rate.sleep()
