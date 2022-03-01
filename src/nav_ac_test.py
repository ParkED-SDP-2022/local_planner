#! /usr/bin/env python3

import rospy
import actionlib
from parked_custom_msgs.msg import NavigateFeedback, NavigateAction, NavigateResult, Point, Robot_Sensor_State, NavigateGoal

def dummyDest():

    dest = Point()
    dest.long = 4.0
    dest.lat = 4.0
    dest.angle = 30

    return dest

def dummyPath():

    p1 = Point()
    p1.long = 1.0
    p1.lat = 1.0
    p1.angle = -999

    p2 = Point()
    p2.long = 2.0
    p2.lat = 2.0
    p2.angle = -999

    p3 = Point()
    p2.long = 2.0
    p2.lat = 2.0
    p2.angle = -999

    path = [p1,p2,p3]
    return path



def navigate_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('bench_x_local_planner', NavigateAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = NavigateGoal(destination = dummyDest(),Path = dummyPath())
    
    ###########################################
    ##### goal.Path not goal.path [CAPS] ######
    ###########################################

    #goal.path = dummyPath()
    #goal.destination = dummyDest()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('navigate_client')
        result = navigate_client()
        print("Result == ", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")