#!/usr/bin/env python

#from UltrasonicControllers.Ultrasonic import UltrasonicSensor
#import MotorDriver

import math
from Contingency import Contingency
import rospy
from parked_custom_msgs import Robot_Sensor_State,Point
from collections import namedtuple
from geometry_msgs import Twist

# location 'struct' for use
Location = namedtuple("Location", "longitude latitude")

class LocalPlanner():

    def __init__(self):
        ## we need the state for our robots

        self.currentLocation = Location(0.0,0.0)
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
    
        self.run_mainflow()

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

        self.currentLocation.longitude = data.longitude
        self.currentLocation.latitude = data.latitude

    def setGlobalPath(self,globalPath):
        self.globalPath = globalPath


    def run_mainflow(self):

        # move from one point to another point
        currentLocIndex = 0
        currentLoc = self.globalPath[0]
        finalLoc = self.globalPath[-1]
        self.myLocation = self.updateLocation()

        while(currentLoc != finalLoc):
            nextLocIndex = currentLocIndex + 1
            nextLoc = self.globalPath[nextLocIndex]
            # need to spin to the next header
            self.spin()
            success = self.moveStraight(currentLoc,nextLoc)

            if (success) : currentLocIndex += 1
            else :
                rospy.loginfo("not successful")
                break
        


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

            #self.motorDriver.setTargetHeading(target_h)
            #self.motorDriver.move()

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







