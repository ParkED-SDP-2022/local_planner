#! /usr/bin/env python3

from doctest import DocTest
import rospy
import actionlib
from parked_custom_msgs.msg import NavigateFeedback, NavigateAction, NavigateResult, Point, Robot_Sensor_State, NavigateGoal

def dummyDest():

    dest = Point()
    dest.long = 357 #900 #950 #-0.5
    dest.lat = 975 #793 #0.5
    dest.angle = 270 #180

    return dest

def dummyDestSim():

    dest = Point()
    dest.long = -0.5 #950 #-0.5
    dest.lat = 0.5 #793 #0.5
    dest.angle = 180 #180

    return dest

def dummyDestSpinCalibration():

    dest = Point()
    dest.long = 775 #950 #-0.5
    dest.lat = 141 #793 #0.5
    dest.angle = 270 #180

    return dest

def dummyDestFullPath():
     
    p9 = Point()
    p9.long = 410 #996
    p9.lat = 200 #357
    p9.angle = 270

    return p9

def dummyPathNoObs():

    #p1 = Point()
    #p1.long = 750 #199
    #p1.lat = 95 #793
    #p1.angle = -999

    p2 = Point()
    p2.long = 750 #500
    p2.lat = 350#793
    p2.angle = -999

    p3 = Point()
    p3.long = 750 #950
    p3.lat = 450 #793
    p3.angle = -999

    p4 = Point()
    p4.long = 750 #908
    p4.lat = 750 #492
    p4.angle = -999

    p5 = Point()
    p5.long = 750 #996
    p5.lat = 950 #357
    p5.angle = -999

    p6 = Point()
    p6.long = 590 #996
    p6.lat = 975 #357
    p6.angle = -999

    p7 = Point()
    p7.long = 357 #996
    p7.lat = 1000 #357
    p7.angle = -999

    #p8 = Point()
    #p8.long =  #996
    #p8.lat =  #357
    #p8.angle = -999

    path = [p3,p4,p5,p6,p7]

    return path

def dummyPathNoObsFull():

    p1 = Point()
    p1.long = 750 #500
    p1.lat = 600#793
    p1.angle = -999

    p2 = Point()
    p2.long = 750 #950
    p2.lat = 930 #793
    p2.angle = -999

    p3 = Point()
    p3.long = 570 #908
    p3.lat = 994 #492
    p3.angle = -999

    p4 = Point()
    p4.long = 240 #996
    p4.lat = 993 #357
    p4.angle = -999

    p5 = Point()
    p5.long = 590 #996
    p5.lat = 975 #357
    p5.angle = -999

    p6 = Point()
    p6.long = 142 #996
    p6.lat = 780 #357
    p6.angle = -999

    p7 = Point()
    p7.long = 140 #996
    p7.lat = 550 #357
    p7.angle = -999

    p8 = Point()
    p8.long = 400 #996
    p8.lat = 498 #357
    p8.angle = -999

    p9 = Point()
    p9.long = 410 #996
    p9.lat = 200 #357
    p9.angle = -999

    path = [p1,p2,p3,p4,p5,p6,p7,p8,p9]

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

def dummyPathWithObs2():

    p1 = Point()
    p1.long = -2
    p1.lat = 0
    p1.angle = -999

    p2 = Point()
    p2.long = -0.5
    p2.lat = 0
    p2.angle = -999

    path = [p1,p2]

    return path

def navigate_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('bench_x_local_planner', NavigateAction)

    print("wait for server")
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("after")

    testDest = dummyDestFullPath()
    testPath = dummyPathNoObsFull()
    # Creates a goal to send to the action server.
    goal = NavigateGoal(destination = testDest,path = testPath)
    
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