#! /usr/bin/env python

import rospy
import actionlib
from parked_custom_msgs.msg import NavigateFeedback, NavigateAction, NavigateResult, Point, Robot_Sensor_State
from LocalPlanner import LocalPlanner

class LocalPlanner(object):

    _feedback = NavigateFeedback()
    _result = NavigateResult()

    def __init__(self, name):
        
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        print('Local Planner Server Starting')
        self._as.start()
        print('Local Planner Server Server started')
        
    
    def execute_cb(self, goal):

        success = True

        goal_pos = goal.destination
        path = goal.path

        self._as.publish_feedback(self._feedback)

        local_planner = LocalPlanner(goal_pos,path)
        success = local_planner.execute_mainflow()
        
        if success:
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('bench_x_local_planner')
    server = LocalPlanner(rospy.get_name())
    rospy.spin()
