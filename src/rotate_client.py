#! /usr/bin/env python
from __future__ import print_function

import sys
import rospy
import actionlib

from actionlib.msg import TestAction, TestActionFeedback, TestActionGoal, TestActionResult


class RotateClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient('action_rotate', TestAction)
        
    def SendGoal(self, goal_val):
        self.client.wait_for_server()
        goal = TestActionGoal(goal = goal_val)
        self.gh = self.client.send_goal(goal, feedback_cb=self.action_feedback)

        self.client.wait_for_result()
        return self.client.get_result()

    def action_feedback(self, fb):
        print(fb.feedback)
        # print(self.client.get_goal_status_text())
        # if(fb.feedback == 3):
        #     self.client.cancel_all_goals()
        #     print("Cancel")

    def shutdown(self):
        rospy.loginfo("Node Shutdown")
        self.client.cancel_all_goals()

if __name__ == '__main__':
    try:
        client = RotateClient()
        rospy.init_node('rotate_client_node')
        rospy.on_shutdown(client.shutdown)  
        result = client.SendGoal(30)
        print(result)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)