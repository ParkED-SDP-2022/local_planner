#!/usr/bin/env python

import math
from Contingency import Contingency
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs import Twist

class LocalPlanner():

    def __init__(self,goal,globalPath):
        ## we need the state for our robots

        self.goal = goal
        self.globalPath = globalPath
        
        self.currentLocation = Point()
        self.currentHeading = 0
        self.usReading = float('inf')
        self.usDistTolerance = 40

        self.twist = Twist()
        self.init_twist()

        ## we need global path
        rospy.init_node('local_planner', anonymous=True)
        rospy.Subscriber("sensor_state", Robot_Sensor_State, self.parse_sensor_state)
        rospy.Subscriber('bench1/gps_pos', Point, self.updateLocation)

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
    
        #rospy.spin()

    def init_twist(self):
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def parse_sensor_state(self,data):
        
        self.currentHeading = data.compass.heading
        self.usReading = data.ultrasonic.distance

    # update current locaton
    def updateLocation(self,data):

        self.currentLocation = data

    def execute_mainflow(self):

        # move from one point to another point
        currentLocIndex = 0
        currentLoc = self.globalPath[0]

        # TODO : define closeTo for Point
        while(currentLoc != self.goal):
            nextLocIndex = currentLocIndex + 1
            nextLoc = self.globalPath[nextLocIndex]
            # need to spin (change heading) to the next location
            self.spin()
            success = self.moveStraight(currentLoc,nextLoc)

            if (success) : currentLocIndex += 1
            else :
                rospy.loginfo("not successful")
                return False
        
        # need to spin to change heading to goal heading
        self.spin(self.goal)
        
        return True
    
    def moveStraight(self,target):
        
        objectDetected = self.scanObstacleUS()
 
        while ( not objectDetected or not self.checkReachTarget(target)):

            # TODO : integrate ros movement
            if (not rospy.is_shutdown()):
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0
                #rospy.loginfo())
                self.cmdvel_pub.publish(self.twist)
                                      

            # target heading calculation
            long1 = self.currentLocation.longitude
            lat1 = self.currentLocation.latitude
            long2 = target.longitude
            lat2 = target.latitude

            y = math.sin(long2-long1)*math.cos(lat2)
            x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1)
            θ = math.atan2(y, x)
            target_h = (θ*180/math.pi + 360) % 360

            # TODO : spin until we reach target heading

            objectDetected = self.scanObstacleUS()

            self.rate.sleep()  
            
        
        if objectDetected :
            # self.motorDriver.motorStop()
            # go to Contingency
            self.stop()
            self.contigency = Contingency()

        
        if self.checkReachTarget(target) : 
            return True
    
    def checkReachTarget(self,target):
        return self.myLocation == target

    def scanObstacleUS(self):
        
        if (self.usReading <= self.usDistTolerance):
            return True
        
        return False

    def stop(self):

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        # rospy.loginfo(hello_str)
        self.cmdvel_pub.publish(self.twist)

    # TODO :: implement actual spin 
    def spin(self,target_heading):

        while(self.currentHeading != target_heading):
            self.twist.linear.x = 0
            self.twist.angular.z = 0.1
            # rospy.loginfo(hello_str)
            self.cmdvel_pub.publish(self.twist)







