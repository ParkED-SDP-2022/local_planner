#! /usr/bin/env python

import rospy
import actionlib
from parked_custom_msgs.msg import NavigateFeedback, NavigateAction, NavigateResult, Point, Robot_Sensor_State

class LocalPlanner(object):

    _feedback = NavigateFeedback()
    _result = NavigateResult()


    def __init__(self, name):
        self._action_name = name
        self._gps_pos = None
        self._gps_pos_sub = rospy.Subscriber('bench1/gps_pos', Point, self.update_current_pos)
        self._sensor_state = None
        self._sensor_state_sub = rospy.Subscriber('sensor_state', Robot_Sensor_State, self.update_sensor_state)
        self._as = actionlib.SimpleActionServer(self._action_name, NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        print('Local Planner Server Starting')
        self._as.start()
        print('Local Planner Server Server started')

    
    def update_current_pos(self, gps_pos):
        self._gps_pos = gps_pos
    
    def update_sensor_state(self, sensor_state):
        self._sensor_state = sensor_state
    
    def execute_cb(self, goal):

        success = True

        goal_pos = goal.destination
        path = goal.path

        self._as.publish_feedback(self._feedback)

        if success:
            self._as.set_succeeded(self._result)



        
    
if __name__ == '__main__':
    rospy.init_node('bench_x_local_planner')
    server = LocalPlanner(rospy.get_name())
    rospy.spin()
