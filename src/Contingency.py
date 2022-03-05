import math
import time
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist
import sys 
# find the gap by figuring out the first consecutive 40 false which indicate where the gap is.
def consecutive_values(input_list):
    n = 0
    count = 0
    for _ in range(2 * len(input_list)):
        if not input_list[n]:
            count += 1
        else:
            count = 0
        if count == 40:
            return n - 39
        n = (n + 1) % len(input_list)
    return -1


class Contingency:

    def __init__(self, triggerDistance, LocalPlanner):
        
        print("Contingency plan initiated")
        self.obstacles = [False for i in range(360)]  # a list of booleans which indicate where the obstacles are
        self.triggerDistance = triggerDistance
        self.closestGap = None
        self.lp = LocalPlanner
        self.usDistTolerance = 0.7

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=10)

    def execute_cont_plan(self):
        self.scanAround()
        self.find_gap()
        self.go()
        sys.exit('exit!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

        return True
    
    def scanAround(self):
        lastheading = round(self.lp.currentHeading)
        # countChanges = 0
        print('scanAround started!!!!!!!!!!!!!!')
        print('----------------------')

        outOfLastHeading = False
        self.lp.stop()
        print(self.obstacles)

        scanned = [False for i in range(0,360)]
        counter = 0
        current_heading = round(self.lp.currentHeading)
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
            
            print(true_count)

        print(self.obstacles)
        # for i in range(0,360):
        #     print('the element in ' ,i, 'th position is ' ,self.obstacles[i])
        self.lp.stop()
        print('scanAround stopped!!!!!!!!!!!!!!!!!!')


    def find_gap(self):
        print('gap is found!!!!!!!!!!')
        # find the gap clockwise
        gap1 = consecutive_values(self.obstacles)

        # find the gap anti-clockwise
        gap2 = consecutive_values(self.obstacles[::1])

        # this returns the gap 1closest to 0 degrees
        if gap2 < gap1:
            self.closestGap = 360 - consecutive_values(self.obstacles[::1])
        else:
            self.closestGap = gap1
        print(self.closestGap)

    def go(self):
        print('go!!!!!!!!!!!1!!!!!!')
        # this finds the target heading for set target heading
        if self.closestGap > 180:
            targetheading = self.closestGap - 20
        else:
            targetheading = self.closestGap + 20
        # this provides parameter for distance calculation
        if targetheading > 180:
            deg = 360 - targetheading
        else:
            deg = targetheading
        distance = self.usDistTolerance / math.cos(math.radians(deg))
        self.lp.twist.linear.x = 0.3
        self.lp.twist.angular.z = 0
        self.cmdvel_pub.publish(self.lp.twist)
        self.rest(abs(distance/self.usDistTolerance))
        self.lp.stop()
    

    def rest(self, t):
        time.sleep(t)     
