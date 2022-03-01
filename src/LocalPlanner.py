#!/usr/bin/env python

import math
from Contingency import Contingency
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class LocalPlanner():

    def __init__(self):
        ## we need the state for our robots
        print("success")
        self.goal = None
        self.globalPath = None
        
        self.currentLocation = Point()
        self.currentHeading = 0
        self.usReading = float('inf')
        self.usDistTolerance = 40

        self.twist = Twist()
        self.init_twist()

        # rospy.init_node('local_planner', anonymous=True)
        #rospy.Subscriber("chatter",String,self.callback) # just a test node

        rospy.Subscriber("sensor_state", Robot_Sensor_State, self.parse_sensor_state)
        rospy.Subscriber('bench1/gps_pos', Point, self.updateLocation)

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
    

#    def callback(self,data):
#        rospy.loginfo(rospy.get_caller_id() + "I heard haha%s", data.data)

    def set_path(self,path):
        self.globalPath = path
    
    def set_goal(self,goal):
        self.goal = goal
    
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

            next_heading = self.calculateTargetHeading(nextLoc)
            self.spin(next_heading)
            success = self.moveStraight(currentLoc,nextLoc)
            print(self.currentLocation)

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
                self.cmdvel_pub.publish(self.twist)
                                      
            objectDetected = self.scanObstacleUS()

            self.rate.sleep()  
            
        
        if objectDetected :
            # self.motorDriver.motorStop()
            # go to Contingency
            self.stop()
            self.contigency = Contingency()

        
        if self.checkReachTarget(target) : 
            return True
    
    def calculateTargetHeading(self,targetPoint):
        # target heading calculation
        long1 = self.currentLocation.long
        lat1 = self.currentLocation.lat
        long2 = targetPoint.long
        lat2 = targetPoint.lat

        y = math.sin(long2-long1)*math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1)
        θ = math.atan2(y, x)
        target_h = (θ*180/math.pi + 360) % 360

        return target_h
        
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
            self.cmdvel_pub.publish(self.twist)







