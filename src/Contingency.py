import math
import time
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist
import sys 


GAP_COUNT = 15

# find the gap by figuring out the first consecutive 40 false which indicate where the gap is.
def consecutive_values(input_list):
    n = 0
    count = 0
    for _ in range(2 * len(input_list)):
        if not input_list[n]:
            count += 1
        else:
            count = 0
        if count == GAP_COUNT:
            return n - (GAP_COUNT)
        n = (n + 1) % len(input_list)
    return -1

# find the gap by figuring out the first consecutive 40 false which indicate where the gap is.
def consecutive_values2(input_list,clockwise,currentHeading):
    
    count = 0

    if clockwise:

        start = round(currentHeading/3.0)
        end = round((currentHeading+90)/3.0)
        for i in range(start,end):
            if not input_list[i]:
                count += 1
            else:
                count = 0
            if count == GAP_COUNT:
                return i - (GAP_COUNT-1)
            #n = (n + 1) % len(input_list)
    else :

        start = round(currentHeading/3.0)
        stop = round((currentHeading-90)/3.0)
        for i in range(start,stop,-1):
            if not input_list[i]:
                count += 1
            else:
                count = 0
            if count == GAP_COUNT:
                return (360-i) % 360
            #n = (n + 1) % len(input_list)
    
    return -1


class Contingency:

    def __init__(self, triggerDistance, LocalPlanner):
        
        print("Contingency plan initiated")
        
        self.obstacles = [False for i in range(180)]  # a list of booleans which indicate where the obstacles are 60 from -90 to + 90 range
        self.virtual_obstacles = []

        self.triggerDistance = triggerDistance
        self.closestGap = None

        self.lp = LocalPlanner
        self.original_heading = round(self.lp.currentHeading)
        self.right_heading = (self.original_heading + 90) % 360
        self.left_heading = (self.original_heading - 90) % 360

        self.usDistTolerance = 0.7

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=10)

    def execute_cont_plan(self):
        
        self.obstacles = [False for i in range(60)]
        
        self.original_heading = round(self.lp.currentHeading)
        self.right_heading = (self.original_heading + 90) % 360
        self.left_heading = (self.original_heading - 90) % 360

        self.scanAround()
        gap_exist = self.find_gap()
        if (not gap_exist): return False
        self.go()
        
        return True
    
    def scanAround(self):
        
        # countChanges = 0
        print('scanAround started!')
        print('----------------------')

        #outOfLastHeading = False
        self.lp.stop()

        #print(self.obstacles)

        #scanned = [False for i in range(0,360)]
        counter = 0
        print('scan spin left')
        # spin left
        while not self.lp.closeToHeading(self.left_heading): 

            self.lp.twist.linear.x = 0
            self.lp.twist.angular.z = self.lp.ANGULAR_SPEED

            current_heading = round(self.lp.currentHeading/3.0)
            self.cmdvel_pub.publish(self.lp.twist)
            #if(self.lp.closeToHeading(current_heading+1)):
            #    self.lp.stop()
            if self.lp.usReadingFront < self.usDistTolerance:
                self.obstacles[current_heading % 60] = True
                # countChanges += 1
            
            #counter += 1
            #current_heading = (current_heading + 1) % 360
            #scanned[current_heading % 60] = True
            #true_count = sum(scanned)
            
            #print(true_count)

        #print(self.obstacles)
        # for i in range(0,360):
        #     print('the element in ' ,i, 'th position is ' ,self.obstacles[i])
        self.lp.stop()
        #self.lp.spin(self.ch)

        print('scan spin right')
        # spin right
        while not self.lp.closeToHeading(self.right_heading): 

            self.lp.twist.linear.x = 0
            self.lp.twist.angular.z = -self.lp.ANGULAR_SPEED

            current_heading = round(self.lp.currentHeading/3.0)
            self.cmdvel_pub.publish(self.lp.twist)
            #if(self.lp.closeToHeading(current_heading+1)):
            #    self.lp.stop()
            if self.lp.usReadingFront < self.usDistTolerance:
                self.obstacles[current_heading % 60] = True
                # countChanges += 1
            
            #counter += 1
            #current_heading = (current_heading + 1) % 360
            #scanned[current_heading % 60] = True
            #true_count = sum(scanned)
            
            #print(true_count)

        #print(self.obstacles)
        # for i in range(0,360):
        #     print('the element in ' ,i, 'th position is ' ,self.obstacles[i])
        self.lp.stop()
        #self.lp.spin(self.ch)

        print('scanAround stop')


    def find_gap(self):
        
        # find the gap clockwise
        gap1 = consecutive_values2(self.obstacles,True,self.original_heading)
        # find the gap anti-clockwise
        gap2 = consecutive_values2(self.obstacles,False,self.original_heading)

        # no gap found
        if gap1 == -1 and gap2 == -1 : return False

        gap1_diff = self.original_heading - gap1
        gap2_diff = 180

        self.gapDir = "right"
        print("gap1 ", gap1 , " gap2 " , gap2 , " ")
        if (gap1_diff < gap2_diff):
            self.closestGap = gap1
            self.lp.spin(self.closestGap + GAP_COUNT/2)
        else : 
            self.closestGap = gap2
            self.lp.spin(self.closestGap - GAP_COUNT/2)
            self.gapDir = "left"

        print('gap is found')
        print(self.closestGap)

        return True     

    def go(self):

        ## TODO : think about the gap 
        
        print('go!!!!!!!!!!!!!!!!!')
        # this finds the target heading for set target heading
        if self.closestGap > 180:
            targetheading = self.closestGap - 20 + self.original_heading
        else:
            targetheading = self.closestGap + 20 + self.original_heading
        # this provides parameter for distance calculation
        if targetheading > 180:
            deg = 360 - targetheading
        else:
            deg = targetheading

        #distance = self.usDistTolerance / math.cos(math.radians(deg))

        detectedObject = False
        # gap on right so we use left sensor , opposite for other dir
        if (self.gapDir == "right"):
            while (self.lp.usReadingLeft < self.usDistTolerance or not detectedObject):
                self.lp.twist.linear.x = 0.05
                self.lp.twist.angular.z = 0
                self.cmdvel_pub.publish(self.lp.twist)

                print("leftDist : ", self.lp.usReadingLeft)
                if (self.lp.scanObstacleUS()):
                    print("front obstacle detected")
                    break
                if self.lp.usReadingLeft < self.usDistTolerance :
                    detectedObject = True

        else :
            while (self.lp.usReadingRight < self.usDistTolerance or not detectedObject):
                self.lp.twist.linear.x = 0.05
                self.lp.twist.angular.z = 0
                self.cmdvel_pub.publish(self.lp.twist)

                if (self.lp.scanObstacleUS()):
                    print("front obstacle detected")
                    break
                
                if self.lp.usReadingRight < self.usDistTolerance :
                    detectedObject = True
            

        #self.rest(abs(distance/self.usDistTolerance))
        self.lp.stop()

        print("go finished")
        return True
    

    def rest(self, t):
        time.sleep(t)     
