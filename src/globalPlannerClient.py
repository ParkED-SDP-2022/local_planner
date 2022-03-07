#! /usr/bin/env python3

from doctest import DocTest
import rospy
import actionlib
from parked_custom_msgs.msg import Point,PlanGlobalPathAction,PlanGlobalPathActionGoal,PlanGlobalPathGoal

class GlobalPlannerClient:

    def  __init__(self):
        self.client = actionlib.SimpleActionClient('plan_global_path', PlanGlobalPathAction)
        
    def replan_path(self,currentNode,obstacleNode,destinationNode):
    
        print("wait for server")
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()
        print("after")

        current_position = currentNode
        destination = destinationNode

        constraint = [currentNode,obstacleNode]
        


        # Creates a goal to send to the action server.
        goal = PlanGlobalPathGoal(current_position = current_position,destination = destination,constraint = constraint)
        
        #goal.path = dummyPath()
        #goal.destination = dummyDest()

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        result = self.client.get_result()

        newGlobalPath = result.path
        # Prints out the result of executing the action
        return newGlobalPath
