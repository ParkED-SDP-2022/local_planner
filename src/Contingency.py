import math
import time
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist
import sys 


GAP_COUNT = 60

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
        for i in range(currentHeading,currentHeading+180):
            if not input_list[i]:
                count += 1
            else:
                count = 0
            if count == GAP_COUNT:
                return i - (GAP_COUNT-1)
            #n = (n + 1) % len(input_list)
    else :

        start = currentHeading 
        stop = (currentHeading-180)
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
        
        self.obstacles = [False for i in range(360)]  # a list of booleans which indicate where the obstacles are
        self.virtual_obstacles = []

        self.triggerDistance = triggerDistance
        self.closestGap = None

        self.lp = LocalPlanner
        self.ch = round(self.lp.currentHeading)

        self.usDistTolerance = 0.7

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=10)

    def execute_cont_plan(self):

        self.ch = round(self.lp.currentHeading)
        self.scanAround()
        self.find_gap()
        self.go()
        sys.exit('exit!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

        return True
    
    def scanAround(self):
        
        # countChanges = 0
        print('scanAround started!!!!!!!!!!!!!!')
        print('----------------------')

        outOfLastHeading = False
        self.lp.stop()
        #print(self.obstacles)

        scanned = [False for i in range(0,360)]
        counter = 0
        current_heading =self.ch
        while not all(scanned): 

            self.lp.twist.linear.x = 0
            self.lp.twist.angular.z = 0.3
            current_heading = round(self.lp.currentHeading)
            self.cmdvel_pub.publish(self.lp.twist)
            if(self.lp.closeToHeading(current_heading+1)):
                self.lp.stop()
            if self.lp.usReading < self.usDistTolerance:
                self.obstacles[current_heading % 360] = True
                # countChanges += 1
            
            counter += 1
            #current_heading = (current_heading + 1) % 360
            scanned[current_heading % 360] = True
            true_count = sum(scanned)
            
            #print(true_count)

        #print(self.obstacles)
        # for i in range(0,360):
        #     print('the element in ' ,i, 'th position is ' ,self.obstacles[i])
        self.lp.stop()
        #self.lp.spin(self.ch)
        print('scanAround stopped! ')


    def find_gap(self):
        #self.virtual_obstacles = [self.obstacles[self.ch:359], self.obstacles[0:self.ch]]
        #print("virtual obstacles")
        #print(self.virtual_obstacles)
        
        # find the gap clockwise
        #gap1 = consecutive_values(self.virtual_obstacles)
        gap1 = consecutive_values2(self.obstacles,True,self.ch)
        # find the gap anti-clockwise
        #gap2 = consecutive_values(self.virtual_obstacles[::1])
        gap2 = consecutive_values2(self.obstacles,False,self.ch)

        # this returns the gap closest to 0 degrees
        #if gap2 < gap1:
        #    self.closestGap = 360 - consecutive_values(self.virtual_obstacles[::1])
        #else:
        #    self.closestGap = gap1

        gap1_diff = self.ch - gap1
        gap2_diff = 180


        print("gap1 ", gap1 , " gap2 " , gap2 , " ")
        if (gap1_diff < gap2_diff):
            self.closestGap = gap1
            self.lp.spin(self.closestGap + GAP_COUNT/2)
        else : 
            self.closestGap = gap2
            self.lp.spin(self.closestGap - GAP_COUNT/2)

        print('gap is found!!!!!!!!!!')
        print(self.closestGap)

        
        

    def go(self):

        ## TODO : need to check if it detects an object then stop moving 
        # maybe better to pass it to the main flow after we get spin to the heading
        
        print('go!!!!!!!!!!!!!!!!!')
        # this finds the target heading for set target heading
        if self.closestGap > 180:
            targetheading = self.closestGap - 20 + self.ch
        else:
            targetheading = self.closestGap + 20 + self.ch
        # this provides parameter for distance calculation
        if targetheading > 180:
            deg = 360 - targetheading
        else:
            deg = targetheading
        distance = self.usDistTolerance / math.cos(math.radians(deg))
        self.lp.twist.linear.x = 0.1
        self.lp.twist.angular.z = 0
        self.cmdvel_pub.publish(self.lp.twist)
        self.rest(abs(distance/self.usDistTolerance))
        self.lp.stop()
    

    def rest(self, t):
        time.sleep(t)     
