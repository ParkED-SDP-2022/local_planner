import math
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

    def __init__(self, triggerDistance, motorDriver):
        self.obstacles = []  # a list of numbers in terms of degree which indicate where the obstacles are
        # self.motorDriver = MotorDriver()
        self.triggerDistance = triggerDistance
        self.closestGap = None

        rospy.Subscriber("sensor_state", Robot_Sensor_State, self.parse_sensor_state)
        rospy.Subscriber('bench1/gps_pos', Point, self.updateLocation)

        self.cmdvel_pub = rospy.Publisher('cmd_vel', Twist , queue_size=10)

        # a gap from 0 to 90, for temporary use
        for i in range(90):
            self.obstacles.append(False)
        for i in range(270):
            self.obstacles.append(True)

    # this method fills 'obstacles' so that we know where they are
    def spin(self):
        for heading in range(90):
            # TODO change it to publish to cmd_vel
            # self.obstacles.append() # if there is obstacle then append true vice versa
            pass

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
            target_h = self.closestGap - 20
        else:
            target_h = self.closestGap + 20
        # this provides parameter for distance calculation
        if target_h > 180:
            deg = 360 - target_h
        else:
            deg = target_h
        
        distance = self.triggerDistance / math.cos(math.radians(deg))

        # TODO change it to publish to cmd_vel
        
