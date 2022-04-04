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
import time

class LocalPlanner():

    def __init__(self):
        ## we need the state for our robots
        print("Local planner initiated")
        self.goal = None
        self.globalPath = None
        
        #self.LEFT_RIGHT_SENSOR = False
        
        self.currentLocation = Point()
        self.currentHeading = 0
        self.usReadingFR = float('inf')
        self.usReadingFL = float('inf')
        
        self.usReadingFront = float('inf')
        self.usReadingBack = float('inf')
        
        #self.usReadingRight = float('inf')
        #self.usReadingLeft = float('inf')

        self.usDistStop = 40

        self.distanceTolerance = 50

        self.degreeTolerance = 5
        self.degreeDelaySpin = 45
        
        self.sec_per_degree_vel = 0.0195388889
        
        self.LINEAR_SPEED = 1 # 1
        self.ANGULAR_SPEED = 1 # 40
        
        self.objectDetected = False

        self.twist = Twist()
        self.init_twist()
        
        self.contigency = Contingency(self.usDistStop,self)
        self.globalPlannerClient = GlobalPlannerClient()

        rospy.Subscriber("/bench_sensor_state", Robot_Sensor_State, self.parse_sensor_state)
        rospy.Subscriber("/robot_position", Point, self.updateLocation)

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=1)

    ## going to the right with 45 delay
    # off by 5.86
    # 
    # going to the left with 45 delay
    # 
    # 
    # #    
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

    #def parse_heading(self,data):
    #    self.currentHeading = data.heading

    def parse_sensor_state(self,data):

        ## if ultrasonic too close it value will be very high
        
        self.usReadingFront= data.UltrasonicFront.distance # demo 2 hardware
        self.usReadingBack = data.UltrasonicBack.distance
        #self.usReadingFRight = data.UltrasonicFront.distance

        #self.usReadingFL = data.ultrasonicFLeft.distance
        
        #self.usReadingLeft = data.UltrasonicLeft.distance
        #self.usReadingRight = data.UltrasonicRight.distance

        #rospy.loginfo("heading: " + str(self.currentHeading) + " us_d: " + str(self.usReading))

    # update current location
    def updateLocation(self,data):

        garbagePoint = Point()
        garbagePoint.lat = -999
        garbagePoint.long = -999

        if (data.lat != garbagePoint.lat and data.long != garbagePoint.long):
            self.currentLocation.lat = data.lat
            self.currentLocation.long = data.long
        if (data.angle != -999):
            self.currentHeading = data.angle
        #rospy.loginfo("Long : " + str(data.long) + " Lat : " + str(data.lat))

    # check if current location is close to a point
    def closeTo(self, p2):
        p1 = self.currentLocation

        # TODO : need to swap
        p1_point = [p1.long,p1.lat]
        p2_point = [p2.long,p2.lat]

        #print("DISTANCE: " ,math.dist(p1_point,p2_point))
        return math.dist(p1_point,p2_point) <= self.distanceTolerance

    def execute_mainflow(self):

        # move from one point to another point
        nextLocIndex = 0
        print("-----------------------------")
        print("goal \n", self.goal)
        print("-----------------------------")
        print("prev global path \n" ,self.globalPath)
        print("-----------------------------")
        #self.callGlobalPlannerTest()
        #print("-----------------------------")
        #print("new global path \n" ,self.globalPath)
        #print("-----------------------------")

        # define closeTo for Point
        while(not self.closeTo(self.goal)):
            
            if (nextLocIndex >= len(self.globalPath)): break
            nextLoc = self.globalPath[nextLocIndex]
            # need to spin (change heading) to the next location
            print("current location")
            print(self.currentLocation)
            print("next location")
            print(nextLoc)
            next_heading = self.true_bearing(self.currentLocation,nextLoc)
            
            print("Spin from current heading : (" ,self.currentHeading,") to next: (",next_heading,")")
            
            while(not self.closeToHeading(next_heading)) : self.spin(next_heading)

            if (not self.closeToHeading(next_heading)):
                diff = self.degree_diff(self.currentHeading,next_heading)
                print("heading not close enough with diff: ", diff)
            
            print("Moving from current location : (" ,self.currentLocation.long,",",self.currentLocation.lat,") to next: (", nextLoc.long,",", nextLoc.lat,")")
            success = self.moveStraight(nextLoc,0)
            
            # object successfully avoided
            # assume that the object are on the next node, so we move on to the next node
            if (success and self.objectDetected):
        
                self.objectDetected = False
                nextLocIndex += 1
                continue

            if (not success and self.objectDetected):

                self.callGlobalPlanner(nextLoc) # update new path
                nextLocIndex = 0 # start from new with new global path
                self.objectDetected = False # reset the object detected boolean
                continue

            nextLocIndex += 1
            
        # need to spin to change heading to goal heading
        self.spin(self.goal.angle)
        self.stop()
        print("Finished executing main loop")
        
        return True
    
    def moveStraight(self,target,targetHeading):
        
        objectDetected = self.scanObstacleUS()
        print("Moving straight")

        if (not self.closeTo(target)):
            self.twist.linear.x = self.LINEAR_SPEED
            self.twist.angular.z = 0
            self.cmdvel_pub.publish(self.twist)

        while (not self.closeTo(target)):

            objectDetected = self.scanObstacleUS()
            
            if objectDetected : 

                print("STOP")
                self.stop()
                print("Object detected, waiting for 10 seconds")
                time.sleep(10)
                objectDetected = self.scanObstacleUS()
                if (not objectDetected) : 
                    self.twist.linear.x = self.LINEAR_SPEED
                    self.twist.angular.z = 0
                    self.cmdvel_pub.publish(self.twist)
                    continue
                
                else :
                    break

            #self.rate.sleep()  
    
        self.stop()
        #time.sleep(1)

        print("Stop")

        if(objectDetected) :
    
            # go to Contingency
            print("Object detected")
            self.objectDetected = True
            avoided = False #self.contigency.execute_cont_plan()
            if (not avoided) : print("FAIL TO AVOID")
            return avoided
           
        if self.checkReachTarget(target) : 
            return True

    def checkReachTarget(self,target):
        return self.closeTo(target)

    def scanObstacleUS(self):
        
        if (self.usReadingFront <= self.usDistStop) : #or (self.usReadingFL <= self.usDistStop):
            return True
        
        return False

    def stop(self):

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        # rospy.loginfo(hello_str)
        self.cmdvel_pub.publish(self.twist)

        time.sleep(1)

    def closeToHeading(self,target_heading):       
        
        diff = self.degree_diff(self.currentHeading,target_heading)
        if diff <= self.degreeTolerance: 
            return True
        else:
            return False

    def spin_dir(self,cur,target):

        if (cur < target) : cur += 360
        left = cur - target

        if (left < 180):
            return "RIGHT"
        else :
            return "LEFT"

    def degree_diff(self,c,t):
        raw_diff = c - t if c > t else t - c
        mod_diff = math.fmod(raw_diff,360)
        dist = 360 - mod_diff if mod_diff > 180 else mod_diff

        return dist
    
    def spin(self,target_heading):
        
        self.spin_time_based(target_heading)
        return True
        
        if (target_heading == -999): return True
        print("CURRENT:" , self.currentHeading , " TARG: " , target_heading)
        
        oppositeSpin = False

        if (not self.closeToHeading(target_heading)):
            
            self.twist.linear.x = 0

            dir = self.spin_dir(self.currentHeading,target_heading)
            
            diff = self.degree_diff(self.currentHeading,target_heading)
            
            
            print("spin dir : ", dir , " angle diff: " , diff)
            if (diff <= self.degreeDelaySpin and dir == "RIGHT"):
                dir = "LEFT"
                oppositeSpin = True
            elif (diff <= self.degreeDelaySpin and dir == "LEFT"):
                dir = "RIGHT"
                oppositeSpin = True
                
            if dir == "RIGHT":
                self.twist.angular.z = self.ANGULAR_SPEED
            else:
                self.twist.angular.z = -self.ANGULAR_SPEED

            self.cmdvel_pub.publish(self.twist) 

        
        while(True):
            
            if (self.closeToHeading(target_heading)) :
                break

            diff = self.degree_diff(self.currentHeading,target_heading)
            if (oppositeSpin == True):
                if (diff > self.degreeDelaySpin):
                    oppositeSpin = False
            
            elif (diff <= self.degreeDelaySpin and oppositeSpin == False):
                break

            #print("target: ", target_heading, " current : ",self.currentHeading, "diff : " , diff)
            
        
        self.stop()

        return True
    
    def spin_time_based(self,target_heading):

        time.sleep(1)

        if (target_heading == -999): return True
        print("CURRENT:" , self.currentHeading , " TARG: " , target_heading)

        time_spin = 0

        if (not self.closeToHeading(target_heading)):
            
            self.twist.linear.x = 0

            dir = self.spin_dir(self.currentHeading,target_heading)
            
            
            diff = self.degree_diff(self.currentHeading,target_heading)
            time_spin = diff * self.sec_per_degree_vel + 10 * self.sec_per_degree_vel

            if dir == "RIGHT":
                self.twist.angular.z = self.ANGULAR_SPEED
            else:
                self.twist.angular.z = -self.ANGULAR_SPEED

            self.cmdvel_pub.publish(self.twist) 

        
        t_end = time.time() + time_spin
        while time.time() < t_end:
            pass
        
        self.stop()

        return True
    
    def true_bearing(self,curLoc,target):

        # long x axis lat y axis
        a1 = curLoc.long
        a2 = curLoc.lat
        
        b1 = target.long
        b2 = target.lat

        if (a1 == b1 and a2 == b2):
            return 0
        
        theta = math.atan2(a1-b1,b2-a2)

        if (theta < 0.0):
            theta += math.pi*2

        return math.degrees(theta)

    def callGlobalPlanner(self, obstacleNode):

        currentNode = self.getCurrentNode(obstacleNode)
        goalNode = self.globalPath[-1]
        #constraint = [currentNode,obstacleNode]

        new_path = self.globalPlannerClient.replan_path(currentNode,obstacleNode,goalNode)

        print("-----------------------------")
        print("new global path \n" ,new_path)
        print("-----------------------------")

        self.set_path(new_path)

    def callGlobalPlannerSync(self):

        currentNode = self.currentLocation
        goalNode = self.goal

        new_path = self.globalPlannerClient.sync_path(currentNode,goalNode)
        self.set_path(new_path)

    def getCurrentNode(self,nextLoc):

        currentNodeIndex = 0

        if len(self.globalPath) > 1:
            for i in range(1,len(self.globalPath)):
                if self.globalPath[i] == nextLoc :
                    currentNodeIndex = i-1
        
        return self.globalPath[currentNodeIndex]