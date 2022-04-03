#! /usr/bin/env python3

from doctest import DocTest
import rospy
import actionlib
import copy
from parked_custom_msgs.msg import Point,PlanGlobalPathAction,PlanGlobalPathActionGoal,PlanGlobalPathGoal

class GlobalPlannerClient:

    def  __init__(self):
        self.LAT_MIN = 0.0
        self.LAT_MAX = 1.2631578947
        self.LONG_MIN = 0.0
        self.LONG_MAX = 1.0
        self.IMAGE_Y = 950
        self.IMAGE_X = 1200

        self.client = actionlib.SimpleActionClient('plan_global_path', PlanGlobalPathAction)
        
    def replan_path(self,currentNode,obstacleNode,destinationNode):
    
        print("wait for server")
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()
        print("after")

        current_position = self.convert_to_longlat(currentNode)
        destination = self.convert_to_longlat(destinationNode)

        constraint = [self.convert_to_longlat(currentNode),self.convert_to_longlat(obstacleNode)]
        
        # Creates a goal to send to the action server.
        goal = PlanGlobalPathGoal(current_position = current_position,destination = destination,constraints = constraint)
        
        #goal.path = dummyPath()
        #goal.destination = dummyDest()

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        result = self.client.get_result()

<<<<<<< HEAD
        newGlobalPathLongLat = result.path
        newGlobalPath = [self.convert_to_longlat(x) for x in newGlobalPathLongLat]
=======
        newGlobalPath = self.convert_to_pixel(result.path,1)
>>>>>>> dceba8c462e2050cc410ebb3c48d4eeaab8ad4a4
        # Prints out the result of executing the action
        return newGlobalPath

    def sync_path(self,currentNode,goalNode):

        print("wait for server")
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()
        print("after")

        current_position = self.convert_to_longlat(currentNode)
        goal = self.convert_to_longlat(goalNode)

        # Creates a goal to send to the action server.
        goal = PlanGlobalPathGoal(current_position = current_position,destination = goal,constraints = [])
        
        #goal.path = dummyPath()
        #goal.destination = dummyDest()

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        result = self.client.get_result()

<<<<<<< HEAD
        newGlobalPathLongLat = result.path
        newGlobalPath = [self.convert_to_longlat(x) for x in newGlobalPathLongLat]
=======
        newGlobalPath = self.convert_to_pixel(result.path,1)
>>>>>>> dceba8c462e2050cc410ebb3c48d4eeaab8ad4a4
        # Prints out the result of executing the action
        return newGlobalPath
        
    def convert_to_longlat(self,position_in_point):

        change_in_Long = self.LONG_MAX - self.LONG_MIN
        change_in_lat = self.LAT_MAX - self.LAT_MIN
        long_conversion_constant = change_in_Long / self.IMAGE_Y
        lat_conversion_constant = change_in_lat / self.IMAGE_X

        point_to_convert = copy.deepcopy(position_in_point)
        
        point_to_convert.long = long_conversion_constant * point_to_convert.long
        point_to_convert.lat = lat_conversion_constant * point_to_convert.lat

        return point_to_convert

    def convert_to_pixel(self,data,flag):
        
        change_in_Long = self.LONG_MAX - self.LONG_MIN
        change_in_lat = self.LAT_MAX - self.LAT_MIN
        image_x = self.IMAGE_X
        image_y = self.IMAGE_Y

        processed_points = []
        for point in data:
            long_conversion_constant = image_y / change_in_Long
            lat_conversion_constant = image_x / change_in_lat

            point_to_convert = point

            if flag == 1:
                point_to_convert.long = long_conversion_constant * point_to_convert.long
                point_to_convert.lat = lat_conversion_constant * point_to_convert.lat
            if flag == 2:
                point_to_convert.long = (1 / long_conversion_constant) * point_to_convert.long
                point_to_convert.lat = (1 / lat_conversion_constant) * point_to_convert.lat

            processed_points.append(point_to_convert)

        return processed_points