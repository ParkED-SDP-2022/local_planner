#!/usr/bin/env python3

import math
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

### node to simulate gps from turtlebot3 sim 

class OdomToGPSConverter:

    def __init__(self):

        rospy.Subscriber("/odom",Odometry,self.subscribe)
        self.gps_pub = rospy.Publisher('bench1/gps_pos',Point,queue_size=10)

        rospy.spin()

    def publish(self,point):
        self.gps_pub.publish(point)


    def subscribe(self,data):

        self.odometry = data
        
        point = Point()
        point.long = data.pose.pose.position.x
        point.lat = data.pose.pose.position.y
        point.angle = -999
        
        self.publish(point)
        

if __name__ == '__main__':
    rospy.init_node("odom_gps_convert")
    test = OdomToGPSConverter()