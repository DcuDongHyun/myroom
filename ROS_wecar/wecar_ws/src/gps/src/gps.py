#!/usr/bin/env python3
import rospy
from serial import Serial
from sensor_msgs.msg import NavSatFix

ser = Serial('/dev/ttyUSB0', 115200)  

rospy.init_node('gps_publisher') 
gps = rospy.Publisher('navsatfix',NavSatFix, queue_size=1) 
rate = rospy.Rate(10)

gps_latitude = None
gps_longitude = None


gps_altitude = None

while not rospy.is_shutdown(): 
    if ser.readable():
        res = ser.readline()
        gps_msg = res.decode()[:len(res) - 1]
        a = gps_msg.split('\n')
        for word in a:
            if 'GNGGA' in word:
                b = word.split(',')
                gps_latitude = float(b[2]) 
                gps_longitude = float(b[4])
                gps_altitude = float(b[9]) 
                msg = NavSatFix()
                msg.latitude = gps_latitude
                msg.longitude = gps_longitude
                msg.altitude = gps_altitude
                gps.publish(msg)
                rate.sleep() 

