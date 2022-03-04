#! /usr/bin/env python3

import rospy
import actionlib
from parked_custom_msgs.msg import NavigateFeedback, NavigateAction, NavigateResult, Point, Robot_Sensor_State, NavigateGoal

def dummyDest():

    dest = Point()
    dest.long = -0.5
    dest.lat = 0.5
    dest.angle = 180

    return dest

def dummyPathNoObs():

    p1 = Point()
    p1.long = -1.0
    p1.lat = -0.5
    p1.angle = -999

    p2 = Point()
    p2.long = -0.5
    p2.lat = -0.5
    p2.angle = -999

    p3 = Point()
    p3.long = -0.5
    p3.lat = 0
    p3.angle = -999

    p4 = Point()
    p4.long = -0.5
    p4.lat = 0.5
    p4.angle = -999

    path = [p1,p2,p3,p4]

    return path

def dummyPathWithObs():

    p1 = Point()
    p1.long = -1.5
    p1.lat = -0.5
    p1.angle = -999

    p2 = Point()
    p2.long = -0.5
    p2.lat = 0.5
    p2.angle = -999

    path = [p1,p2]

    return path


def navigate_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('bench_x_local_planner', NavigateAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    testPath = dummyPathWithObs()
    # Creates a goal to send to the action server.
    goal = NavigateGoal(destination = dummyDest(),Path = testPath)
    
   

    
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