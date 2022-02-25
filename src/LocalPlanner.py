import MotorDriver
import math
import Contingency
from UltrasonicControllers.Ultrasonic import UltrasonicSensor

class LocalPlanner():


    def __init__(self):
        ## we need the state for our robots
        ## we need global path

        self.motorDriver = MotorDriver()
        self.ultrasonic = UltrasonicSensor()

        self.usDistTolerance = 40

        self.myLocation = None

    
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
            self.moveToPoint(currentLoc,nextLoc)
        


    def moveToPoint(self,target):
        
        objectDetected = self.scanObstacleUS()

        while ( not objectDetected or self.checkReachTarget(target)):

            #self.motorDriver.setDistance(distance)
            self.motorDriver.setSpeed(50)
            #self.motorDriver.setHeading(0)

            # TODO : target heading calculation
            self.motorDriver.setTargetHeading(target_h)
            self.motorDriver.move()

            objectDetected = self.scanObstacleUS()
            self.updateLocation()
            
        
        if objectDetected :
            self.motorDriver.motorStop()
            # go to Contingency
        #if reach point2 : True
        pass
    
    def checkReachTarget(self,target):
        return self.myLocation != target

    # update current locaton
    def updateLocation(self):

        self.myLocation = None
        pass

    def scanObstacleUS(self):
        
        distance_forward = self.ultrasonic.distanceForward()
        distance_backward = self.ultrasonic.distanceBackward()
        
        if (distance_forward <= self.usDistTolerance 
            or distance_backward <= self.usDistTolerance):
            return True
        
        return False




