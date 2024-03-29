#!/usr/bin/env python3

import math
import rospy
from parked_custom_msgs.msg import Robot_Sensor_State,Compass,Ultrasonic_Sensor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

### node to simulate gps from turtlebot3 sim 

class SensorSimulation:

    def __init__(self):

        self.scan = LaserScan()
        self.odom = Odometry()

        rospy.Subscriber("/scan",LaserScan,self.subscribeScan)
        rospy.Subscriber("/odom",Odometry,self.subscribeOdom)
        self.sensor_pub = rospy.Publisher('/bench_sensor_state',Robot_Sensor_State,queue_size=10)

        self.publish_loop()

    def publish_loop(self):
        while(not rospy.is_shutdown()):
            self.publish()
    
    def publish(self):

        us_readings = self.scan.ranges
        us_value = float('inf')
        us_back_val = float('inf')
        us_left_val = float('inf')
        us_right_val = float('inf')

        #print(len(us_readings))
        if (len(us_readings) > 0 ): 
            us_value = us_readings[0]
            us_back_val = us_readings[180]
            us_left_val = us_readings[90]
            us_right_val = us_readings[270]

        us_front = Ultrasonic_Sensor()
        us_front.distance = us_value

        us_back = Ultrasonic_Sensor()
        us_back.distance = us_back_val

        us_left = Ultrasonic_Sensor()
        us_left.distance = us_left_val

        us_right = Ultrasonic_Sensor()
        us_right.distance = us_right_val

        (roll, pitch, yaw) = euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, 
            self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])


        yaw_deg = math.degrees(yaw)

        heading = 0 
        if (yaw_deg > 90) :
            gamma = yaw_deg - 90
            heading = 360 - gamma
        else : 
            heading = 90 - yaw_deg
        
        #print("yaw : ", yaw_deg)
        #print("heading: ",heading)
        compass = Compass()
        compass.heading = heading

        sensor_data = Robot_Sensor_State()
        sensor_data.Compass = compass
        sensor_data.UltrasonicFront = us_front
        #sensor_data.ultrasonicFLeft = us_front
        sensor_data.UltrasonicBack = us_back
        sensor_data.UltrasonicLeft = us_left
        sensor_data.UltrasonicRight = us_right


        self.sensor_pub.publish(sensor_data)

    def subscribeOdom(self,data):
        self.odom = data

    def subscribeScan(self,data):

        self.scan = data 

if __name__ == '__main__':
    rospy.init_node("sensor_sim")
    test = SensorSimulation()