#! /usr/bin/env python3

import rospy
import actionlib
from parked_custom_msgs.msg import NavigateFeedback, NavigateAction, NavigateResult, Point, Robot_Sensor_State
from LocalPlanner import LocalPlanner

class ActionServer(object):

    _feedback = NavigateFeedback()
    _result = NavigateResult()

    def __init__(self, name):
        
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        print('Local Planner Server Starting' + " " + name)
        self._as.start()
        print('Local Planner Server Server started')

        self.local_plan = LocalPlanner()
        
    
    def execute_cb(self, goal):

        success = True

        goal_pos = goal.destination
        path = goal.path

        self._as.publish_feedback(self._feedback)

        self.local_plan.set_goal(goal_pos)
        self.local_plan.set_path(path)

        success = self.local_plan.execute_mainflow()
        
        self._result.success = success
        if success:
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('bench_x_local_planner')
    server = ActionServer(rospy.get_name())
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
