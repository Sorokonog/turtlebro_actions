import rospy
import actionlib
import time

from math import degrees
from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion
from actionlib.msg import TestAction, TestActionFeedback, TestActionGoal, TestActionResult
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry

class RotateServer(object):

    _feedback = TestActionFeedback()
    _result   = TestActionResult()

    def __init__(self):

        rospy.init_node('rotate_server_node')
        rospy.on_shutdown(self.shutdown)   
        rospy.loginfo("Start rotation ActionSever")
        self.odom = Odometry()
        self.started = False

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odometry_cb)

        self._as = actionlib.SimpleActionServer('action_rotate', TestAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def odometry_cb(self, msg):
        if self.started:
            self.odom = msg
            self.rotation_odom_loop()

    def execute_cb(self, goal):
        self.rotation_target = goal.goal
        self.start_rotation()
        while self.started:
            rospy.sleep(0.5)


    def start_rotation(self):
        self.started = True 
        self.degree_delta = 0
        self.prev_pose = self.odom

        rospy.loginfo("Start Rotate Action with %i"%self.rotation_target)  
        rotate_cmd = Twist()

        if(self.rotation_target  >0):
            rotate_cmd.angular.z = -0.5
        else: rotate_cmd.angular.z = 0.5   

        self.cmd_vel.publish(rotate_cmd)

    def rotation_odom_loop(self):
    
        self.degree_delta += self.get_degree_diff(self.prev_pose, self.odom)
        self.prev_pose = self.odom

        self._result.result = self.degree_delta

        if (self.is_reach_goal()):
            self.cmd_vel.publish(Twist())
            self._as.set_succeeded(self._result)  
            rospy.loginfo("succeeded")
            self.started = False 
        else :
            self._feedback.feedback = self._result.result
            self._as.publish_feedback(self._feedback)

        if self._as.is_preempt_requested():
            rospy.loginfo('Preempted')
            self.cmd_vel.publish(Twist())
            self._as.set_preempted(self._result)
            self.started = False

        rospy.loginfo("Have degree %s" %self.degree_delta)


    def is_goal_reached(self):

        if(self.rotation_target >=0):
            if(self.degree_delta >= self.rotation_target):
                return True        
            else : return False

        if(self.rotation_target < 0):
            if(self.degree_delta < self.rotation_target):
                return True        
            else : return False


    def get_degree_diff(self, prev, current):

        prev_q = [prev.pose.pose.orientation.x, prev.pose.pose.orientation.y, prev.pose.pose.orientation.z, prev.pose.pose.orientation.w]
        current_q = [current.pose.pose.orientation.x, current.pose.pose.orientation.y, current.pose.pose.orientation.z, current.pose.pose.orientation.w]

        delta_q = quaternion_multiply(prev_q, quaternion_inverse(current_q))    
        (_, _, yaw) = euler_from_quaternion(delta_q)

        return degrees(yaw)        

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        if(self._as.is_active()):
            self.cmd_vel.publish(Twist())
            self._as.set_aborted(self._result)
            rospy.sleep(1)  

if __name__ == '__main__':         
    server = RotateServer()  
    rospy.spin()