#!/usr/bin/env python

import math
from re import S
from Contingency import Contingency
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class LocalPlanner():

    def __init__(self):
        ## we need the state for our robots
        print("Local planner initiated")
        self.goal = None
        self.globalPath = None
        
        self.currentLocation = Point()
        self.currentHeading = 0
        self.usReading = float('inf')

        self.usDistTolerance = 0.5

        self.distanceTolerance = 0.1
        self.degreeTolerance = 0.3
        
        self.LINEAR_SPEED = 0.1
        

        self.twist = Twist()
        self.init_twist()
        
        self.contigency = Contingency(self.usDistTolerance,self)

        # rospy.init_node('local_planner', anonymous=True)
        #rospy.Subscriber("chatter",String,self.callback) # just a test node

        rospy.Subscriber("/bench_sensor_state", Robot_Sensor_State, self.parse_sensor_state)
        rospy.Subscriber('/robot_position', Point, self.updateLocation)

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=10)
        #self.rate = rospy.Rate(50) # 10hz
    

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
        
        self.currentHeading = data.heading.heading
        self.usReading = data.ultrasonicFront.distance # only the front need 3 more readings
        self.usReadingBack = data.ultrasonicBack.distance
        #rospy.loginfo("heading: " + str(self.currentHeading) + " us_d: " + str(self.usReading))

    # update current location
    def updateLocation(self,data):

        self.currentLocation = data
        #rospy.loginfo("Long : " + str(data.long) + " Lat : " + str(data.lat))

    # check if current location is close to a point
    def closeTo(self, p2):
        p1 = self.currentLocation

        p1_point = [p1.long,p1.lat]
        p2_point = [p2.long,p2.lat]

        #print("DISTANCE: " ,math.dist(p1_point,p2_point))
        return math.dist(p1_point,p2_point) <= self.distanceTolerance

    def execute_mainflow(self):

        # move from one point to another point
        nextLocIndex = 0
        print("GLOBAL " ,self.globalPath)
        # define closeTo for Point
        while(not self.closeTo(self.goal)):
            
            nextLoc = self.globalPath[nextLocIndex]
            # need to spin (change heading) to the next location
            print(nextLoc)
            #next_heading = self.calculateTargetHeading(nextLoc)
            next_heading = self.true_bearing(self.currentLocation,nextLoc)
            
            self.spin(next_heading)
            success = self.moveStraight(nextLoc)
            print("CURRENTLOC" ,self.currentLocation)

            if (success) : nextLocIndex += 1
            else :
                print("not success")
                return False
        
        # need to spin to change heading to goal heading
        self.spin(self.goal.angle)

        print("finished main loop")
        
        return True
    
    def moveStraight(self,target):
        
        objectDetected = self.scanObstacleUS()
        print("moving straight")
        while (not self.closeTo(target)):
   
            self.twist.linear.x = self.LINEAR_SPEED
            self.twist.angular.z = 0
            self.cmdvel_pub.publish(self.twist)
                                      
            objectDetected = self.scanObstacleUS()
            if objectDetected : break

            #self.rate.sleep()  
    
        self.stop()

        print("stopped")
        while(objectDetected) :
            # self.motorDriver.motorStop()
            # go to Contingency
            print("Object DETECTED")
            avoided = self.contigency.execute_cont_plan()

            print("finished cont")
            return avoided

        
        if self.checkReachTarget(target) : 
            return True
    
    def calculateTargetHeading(self,targetPoint):
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
        return self.closeTo(target)

    def scanObstacleUS(self):
        
        if (self.usReading <= self.usDistTolerance):
            return True
        
        return False

    def stop(self):

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        # rospy.loginfo(hello_str)
        self.cmdvel_pub.publish(self.twist)

    def closeToHeading(self,target_heading):
        return abs(self.currentHeading-target_heading) <= self.degreeTolerance

    def spin(self,target_heading):
        
        if (target_heading == -999): return True
        print("CURRENT:" , self.currentHeading , " TARG: " , target_heading)
        # self.currentHeading != target_heading
        while(not self.closeToHeading(target_heading)):
            # if heading_difference below 180, target is to the left; above 180, target to the right
            #self.currentHeading-target_heading 
            heading_difference = (target_heading - self.currentHeading) % 360
            
            self.twist.linear.x = 0

            if heading_difference < 180:
                self.twist.angular.z = -0.1
            else:
                self.twist.angular.z = 0.1

            self.cmdvel_pub.publish(self.twist)

            #print("CUR:" , self.currentHeading , " targ:" ,target_heading)
        
        self.stop()

        return True

    def true_bearing(self,loc1,loc2):
        startLat = math.radians(loc1.lat)
        startLong = math.radians(loc1.long)
        endLat = math.radians(loc2.lat)
        endLong = math.radians(loc2.long)

        dLong = endLong - startLong

        dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
        if abs(dLong) > math.pi:
            if dLong > 0.0:
                dLong = -(2.0 * math.pi - dLong)
            else:
                dLong = (2.0 * math.pi + dLong)

        bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0

        return bearing





