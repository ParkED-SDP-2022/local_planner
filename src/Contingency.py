import math
import time
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist
import sys 


GAP_COUNT = 20

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
                return 3.0*(i - (GAP_COUNT-1))

            # i is the last index of the gap
            # so we multiply by 3 to get angle
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
                return ((180-i) % 180) * 3.0
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
        
        self.obstacles = [False for i in range(180)]
        
        self.original_heading = round(self.lp.currentHeading)

        ## need to spin extra because of delay
        # right spin actually goes from 0 -> 270
        # left spin from 0 -> 90

        self.right_heading = (self.original_heading - 90 - self.lp.degreeDelaySpin) % 360
        self.left_heading = (self.original_heading + 90 + self.lp.degreeDelaySpin) % 360

        self.scanAround()
        gap_exist = self.find_gap()
        if (not gap_exist): 
            print("NO GAP FOUND")
            return False
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
        countChanges = 0
        print('scan spin left')
        # spin left

        self.lp.twist.linear.x = 0
        self.lp.twist.angular.z = self.lp.ANGULAR_SPEED
        self.cmdvel_pub.publish(self.lp.twist)

        while not self.lp.closeToHeading(self.left_heading): 

            current_heading = self.lp.currentHeading - self.lp.degreeDelaySpin

            value_heading = round(current_heading/3.0)
            
            if self.lp.usReadingFront < self.usDistTolerance:
                self.obstacles[value_heading % 60] = True
                countChanges += 1
            #counter += 1
            #current_heading = (current_heading + 1) % 360
            #scanned[current_heading % 60] = True
            #true_count = sum(scanned)
            
            #print(true_count)

        print("number of changes:",countChanges)
    
        self.lp.stop()
        
        print('scan spin right')
        self.lp.twist.linear.x = 0
        self.lp.twist.angular.z = -self.lp.ANGULAR_SPEED
        self.cmdvel_pub.publish(self.lp.twist)

        # spin right
        while not self.lp.closeToHeading(self.right_heading): 

            current_heading = self.lp.currentHeading + self.lp.degreeDelaySpin

            value_heading = round(current_heading/3.0)

            if self.lp.usReadingFront < self.usDistTolerance:
                self.obstacles[value_heading % 60] = True
                countChanges += 1
            
            #counter += 1
            #current_heading = (current_heading + 1) % 360
            #scanned[current_heading % 60] = True
            #true_count = sum(scanned)
            
            #print(true_count)

        print("number of changes:",countChanges)
        #print(self.obstacles)
        # for i in range(0,360):
        #     print('the element in ' ,i, 'th position is ' ,self.obstacles[i])
        self.lp.stop()
        #self.lp.spin(self.ch)

        print('scanAround stop')


    def find_gap(self):
        
        # find the gap clockwise (to the right)
        gap1 = consecutive_values2(self.obstacles,True,self.original_heading)
        # find the gap anti-clockwise (to the left)
        gap2 = consecutive_values2(self.obstacles,False,self.original_heading)

        # no gap found
        if gap1 == -1 and gap2 == -1 : return False

        gap1_diff = self.original_heading - gap1
        gap2_diff = 180

        self.gapDir = "right"
        print("gap1 ", gap1 , " gap2 " , gap2 , " ")
        if (gap1_diff < gap2_diff):
            self.closestGap = gap1
            self.lp.spin(self.closestGap + round(GAP_COUNT*3.0/2))
        else : 
            self.closestGap = gap2

            self.lp.spin(self.closestGap - round(GAP_COUNT*3.0/2))
            self.gapDir = "left"

        print('gap is found')
        
        print("Gap: " , self.closestGap)
        print("Gap direction: " , self.gapDir)

        return True     

    def go(self):

        ## TODO : think about the gap 
        
        print('go!!!!!!!!!!!!!!!!!')
        

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
