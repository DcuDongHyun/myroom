#!/usr/bin/python

import rospy
import random
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node("tf_test")

    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        msg = PoseStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.position.x = -1.3 
        msg.pose.position.y = -0.2
        msg.pose.position.z = 0

        quat = quaternion_from_euler(
            0., 0., math.radians(random.randint(0, 180)))

 
        msg.pose.orientation.z = 0.94
        msg.pose.orientation.w =0.3
        pub.publish(msg)

        r.sleep()
