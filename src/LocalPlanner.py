#!/usr/bin/env python

#from UltrasonicControllers.Ultrasonic import UltrasonicSensor
#import MotorDriver

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
        self.currentLocIndex = 0
        currentLoc = self.globalPath[0]
        finalLoc = self.globalPath[-1]
        self.myLocation = self.updateLocation()

        while(currentLoc != finalLoc):
            nextLocIndex = self.currentLocIndex + 1
            nextLoc = self.globalPath[nextLocIndex]
            # need to spin to the next header
            self.spin()
            success = self.moveStraight(currentLoc,nextLoc)
        


    def moveStraight(self,target):
        
        objectDetected = self.scanObstacleUS()
 
        while ( not objectDetected or self.checkReachTarget(target)):

            # TODO : integrate ros movement
            if (not rospy.is_shutdown()):
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0
                #rospy.loginfo())
                self.cmdvel_pub.publish(self.twist)
                                      

            # TODO : target heading calculation
            #self.motorDriver.setTargetHeading(target_h)
            #self.motorDriver.move()

            objectDetected = self.scanObstacleUS()

            self.rate.sleep()  
            
        
        if objectDetected :
            # self.motorDriver.motorStop()
            # go to Contingency
            self.contigency = Contingency()

        
        if self.checkReachTarget(target) : 
            return True
    
    def checkReachTarget(self,target):
        return self.myLocation != target

    def scanObstacleUS(self):
        
        if (self.usReading <= self.usDistTolerance):
            return True
        
        return False

    def spin(self):

        if (not rospy.is_shutdown()):
            self.twist.linear.x = 0
            self.twist.angular.y = 0.1
            # rospy.loginfo(hello_str)
            self.cmdvel_pub.publish(self.twist)







