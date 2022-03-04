import math
import time
import rospy
from parked_custom_msgs.msg import Point, Robot_Sensor_State
from geometry_msgs.msg import Twist

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
        self.obstacles = [True for i in range(6)]  # a list of booleans which indicate where the obstacles are
        self.triggerDistance = triggerDistance
        self.closestGap = None
        self.lp = LocalPlanner

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=10)

    def execute_cont_plan(self):
        self.scanAround()
        self.find_gap()
        self.go()

        return True
    
    def scanAround(self):
        lastheading = self.lp.currentHeading
        # can it spin by 1 degree to obtain 360 values
        for heading in range(360):
            self.lp.twist.angular.z = 0.3
            self.cmdvel_pub.publish(self.lp.twist)
            if(self.lp.currentHeading == 1 + lastheading):
                self.lp.stop()
            if self.lp.usReading < 0.4:
                self.obstacles[round(self.lp.currentHeading + heading)%360] = True
            lastheading = self.lp.currentHeading

    def find_gap(self):
        # find the gap clockwise
        gap1 = consecutive_values(self.obstacles)

        # find the gap anti-clockwise
        gap2 = consecutive_values(self.obstacles[::1])

        # this returns the gap closest to 0 degrees
        if gap2 < gap1:
            self.closestGap = 360 - consecutive_values(self.obstacles[::1])
        else:
            self.closestGap = gap1

    def go(self):
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
        distance = 0.4 / math.cos(math.radians(deg))
        self.lp.twist.linear.x = 0.3
        self.lp.twist.angular.z = 0
        self.cmdvel_pub.publish(self.lp.twist)
        self.rest(distance/0.3)
        self.lp.stop()
    

    def rest(self, t):
        time.sleep(t)     
