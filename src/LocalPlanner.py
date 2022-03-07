#!/usr/bin/env python3

from calendar import c
import math
from re import S
from Contingency import Contingency
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from globalPlannerClient import GlobalPlannerClient

class LocalPlanner():

    def __init__(self):
        ## we need the state for our robots
        print("Local planner initiated")
        self.goal = None
        self.globalPath = None
        
        #self.LEFT_RIGHT_SENSOR = False
        
        self.currentLocation = Point()
        self.currentHeading = 0
        self.usReading = float('inf')

        self.usDistStop= 0.5

        self.distanceTolerance = 10 # 0.2
        self.degreeTolerance = 5 #0.3
        
        self.LINEAR_SPEED = 110 # 0.1
        self.ANGULAR_SPEED = 0.5 # 0.1
        
        self.objectDetected = False

        self.twist = Twist()
        self.init_twist()
        
        # self.contigency = Contingency(self.usDistStop,self)
        self.globalPlannerClient = GlobalPlannerClient()

        # rospy.init_node('local_planner', anonymous=True)
        #rospy.Subscriber("chatter",String,self.callback) # just a test node

        rospy.Subscriber("/bench_sensor_state", Robot_Sensor_State, self.parse_sensor_state)
        rospy.Subscriber("/robot_position", Point, self.updateLocation)

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=1)
        self.rate = rospy.Rate(50) # 5hz
    

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

        ## if ultrasonic too close it value will be very high
        self.currentHeading = data.heading.heading
        self.usReadingFR = data.ultrasonicFRight.distance # demo 2 hardware
        self.usReadingFL = data.ultrasonicFLeft.distance
        self.usReadingBack = data.ultrasonicBack.distance
        
        #self.usReadingLeft = data.ultrasonicLeft.distance
        #self.usReadingRight = data.ultrasonicRight.distance

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
            next_heading = self.true_bearing(self.currentLocation,nextLoc)
            
            self.spin(next_heading)
            success = self.moveStraight(nextLoc)
            print("CURRENTLOC" ,self.currentLocation)

            if (not success and self.objectDetected):

                self.callGlobalPlanner(nextLoc) # update new path
                nextLocIndex = 0 # start from new with new global path
                self.objectDetected = False # reset the object detected boolean

            if (success) : nextLocIndex += 1
            else :
                print("not success")
                return False
        
        # need to spin to change heading to goal heading
        self.spin(self.goal.angle)

        print("finished main loop")
        
        return True
    
    def moveStraight(self,target,targetHeading):
        
        objectDetected = self.scanObstacleUS()
        print("moving straight")
        while (not self.closeTo(target)):
   
            self.twist.linear.x = self.LINEAR_SPEED
            self.twist.angular.z = 0
            self.cmdvel_pub.publish(self.twist)
                                      
            objectDetected = self.scanObstacleUS()
            outOfDistanceTolerance = not self.closeToHeading(targetHeading)

            if (outOfDistanceTolerance) :
                self.spin(targetHeading)
            if objectDetected : break

            #self.rate.sleep()  
    
        self.stop()

        print("stopped")
        if(objectDetected) :
            # self.motorDriver.motorStop()
            # go to Contingency
            print("Object DETECTED")
            self.objectDetected = True
            #avoided = self.contigency.execute_cont_plan()
            
            return False
            print("finished cont")
            self.callGlobalPlanner(target)
            #return avoided

        
        if self.checkReachTarget(target) : 
            return True

    def checkReachTarget(self,target):
        return self.closeTo(target)

    def scanObstacleUS(self):
        
        if (self.usReadingFR <= self.usDistStop) or (self.usReadingFL <= self.usDistStop):
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
            # if heading_difference below 180, target is to the right; above 180, target to the left
            heading_difference = (target_heading - self.currentHeading) % 360
            
            self.twist.linear.x = 0

            if heading_difference < 180:
                self.twist.angular.z = self.ANGULAR_SPEED
            else:
                self.twist.angular.z = -self.ANGULAR_SPEED

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

    def callGlobalPlanner(self, obstacleNode):

        currentNode = self.getCurrentNode(obstacleNode)
        constraint = [currentNode,obstacleNode]

        new_path = self.globalPlannerClient.replan_path(currentNode,self.goal,constraint)

        self.set_path(new_path)


    def getCurrentNode(self,nextLoc):

        currentNodeIndex = 0

        if len(self.globalPath) > 1:
            for i in range(1,len(self.globalPath)):
                if self.globalPath[i] == nextLoc :
                    currentNodeIndex = i-1
        
        return self.globalPath[currentNodeIndex]