#!/usr/bin/env python

#from UltrasonicControllers.Ultrasonic import UltrasonicSensor
#import MotorDriver



from Contingency import Contingency
import rospy
from parked_custom_msgs import Robot_Sensor_State,Point
from collections import namedtuple
from std_msgs import String

# location 'struct' for use
Location = namedtuple("Location", "longitude latitude")



class LocalPlanner():

    def __init__(self):
        ## we need the state for our robots

        self.currentLocation = Location(0.0,0.0)
        self.currentHeading = 0
        self.usReading = float('inf')
        self.usDistTolerance = 40

        ## we need global path
        rospy.init_node('local_planner', anonymous=True)
        rospy.Subscriber("sensor_state", Robot_Sensor_State, self.parse_sensor_state)
        rospy.Subscriber('bench1/gps_pos', Point, self.updateLocation)

        self.cmdvel_pub = rospy.Publisher('cmd_vel', String, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
    
        self.run_mainflow()


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
            success = self.moveToPoint(currentLoc,nextLoc)
        


    def moveToPoint(self,target):
        
        objectDetected = self.scanObstacleUS()

        while ( not objectDetected or self.checkReachTarget(target)):

            # TODO : integrate ros movement
            if (not rospy.is_shutdown()):
                hello_str = "hello world %s" % rospy.get_time()
                rospy.loginfo(hello_str)
                self.cmdvel_pub.publish(hello_str)
                                      

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






