#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan 

def callback(data):
	print (data.ranges)

rospy.init_node('topic_subscriber')

sub=rospy.Subscriber('/scan',LaserScan,callback)

rospy.spin()
