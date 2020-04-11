import rospy
import actionlib
import time

from turtlebro_actions.msg import movetimeGoal, movetimeResult, movetimeFeedback
from turtlebro_actions.msg import movetodistanceGoal, movetodistanceResult, movetodistanceFeedback, movetodistanceAction
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node('move_server_node')
rospy.loginfo("Start TurtleBro ActionSever")

result = movetodistanceResult()
result.done = 1

def execute_cb(goal):
    print(goal)
    tbasm.set_succeeded(result)

tbasm = actionlib.SimpleActionServer('movetodistance', movetodistanceAction, execute_cb, auto_start = False)
tbasm.start()
print("server started")

if __name__ == '__main__':          
    rospy.spin()