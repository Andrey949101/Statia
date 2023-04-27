#!/usr/bin/env python
# Здесь была лицензия =)
#!/usr/bin/env python
import rospy
import tf
import math
from tf.transformations import quaternion_from_euler
from turtlesim.msg import Pose

def handle_carrot_pose(msg):
    angle=int((time.time()*1000))%360
    x=1*math.cos(angle*0.0174533)
    y=1*math.sin(angle*0.0174533)
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0), quaternion_from_euler(0, 0, msg.theta), rospy.Time.now(), turtlename, "world")#
    br.sendTransform((x, y, 0), quaternion_from_euler(0, 0, 0), rospy.Time.now(), turtlename, "turtle1")

if __name__ == '__main__':
    rospy.init_node('carrot')
    turtlename = rospy.get_param('~turtle_tf_name')
    rospy.Subscriber('input_pose', Pose, handle_carrot_pose)
rospy.spin()
