import rospy
import actionlib
import math

from turtlebro_actions.msg import movetimeGoal, movetimeResult, movetimeFeedback
from turtlebro_actions.msg import movetodistanceGoal, movetodistanceResult, movetodistanceFeedback, movetodistanceAction
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from turtlesim.msg import Pose #TOBE removed after testing

class MovetodistanceServer(object):

    def __init__(self):

        #Variables init
        # self._odom = Odometry() #Don't froget to edit after testing to "/cmd_vel"
        self._odom = Pose()
        # self._startPosition = Odometry() #Don't froget to edit after testing to "/cmd_vel"
        self._startPosition = Pose()
        rospy.loginfo("TurtleBro ActionSever vars init complete") #TOREMOVE
        #ROS init
        self._goalRecieved = False
        rospy.init_node('move_server_node')
        rospy.loginfo("Start TurtleBro ActionSever")
        rospy.on_shutdown(self.shutdown)
        self._cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1) #Don't froget to edit after testing to "/cmd_vel"
        # rospy.Subscriber("/odom", Odometry, self.odometry_cb)  ---- to modyfy after tests
        rospy.Subscriber('/turtle1/pose', Pose, self.odometry_cb)  # ---- TO REMOVE after tests
        self._tbasm = actionlib.SimpleActionServer('movetodistance', movetodistanceAction, execute_cb=self.actionserver_goal_recieve_cb, auto_start = False)
        self._tbasm.start()
        rospy.loginfo("TurtleBro ActionSever started") #TOREMOVE
        #Action server inits
        self._result = movetodistanceResult()
        self._feedback = movetodistanceFeedback()
        rospy.loginfo("TurtleBro ActionSever init complete") #TOREMOVE

        
    def odometry_cb(self, msg):
        self._odom = msg
        if self._goalRecieved:
            self.control_loop()

    def actionserver_goal_recieve_cb(self, goal):

        if self._tbasm.is_preempt_requested():
            self.cmd_vel.publish(Twist())
            self._result = None
            self._tbasm.set_preempted(self._result)
            rospy.loginfo('Goal preempted') 
            self._goalRecieved = False

        if (goal.distance <= 0):
            self._goalSpeed = -goal.speed
        else:
            self._goalSpeed = goal.speed
        self._goalDistance = goal.distance
        self._startPosition = self._odom
        self._goalRecieved = True
        rospy.loginfo("Goal recieved")
        print(goal.speed, goal.distance) #TOREMOVE
        while self._goalRecieved:
            rospy.sleep(0.5)

    def move_control_function(self, velx):

        cmd_vel_Twist = Twist()
        cmd_vel_Twist.linear.x = velx
        self._cmd_vel.publish(cmd_vel_Twist)


    def control_loop(self):
        #rospy.loginfo("Goal processing") 
        self._feedback.distance_passed = abs(self._odom.x) - abs(self._startPosition.x) # TOREMOVE after tests
        #self._feedback.distance_passed = abs(self._odom.pose.pose.position.x) - abs(self._startPosition.pose.pose.position.x)

        if self.is_goal_reached():
            self._cmd_vel.publish(Twist())
            self._result.done = True
            self._tbasm.set_succeeded(self._result)
            rospy.loginfo("Goal reached")
            print("START ", self._startPosition.x)
            print("STOP ",self._odom.x)
            self._goalRecieved = False
        else:
            self.move_control_function(self._goalSpeed)
            self._tbasm.publish_feedback(self._feedback)
            #rospy.loginfo("FEEDBACK")

    
    def is_goal_reached(self):

        #if(abs(abs(self._odom.pose.pose.position.x) - abs(self._startPosition.pose.pose.position.x)) <= abs(self._goalDistance)): 
        if(abs(abs(self._odom.x) - abs(self._startPosition.x)) <= abs(self._goalDistance)):
            return False
        else:
            print(abs(abs(self._odom.x) - abs(self._startPosition.x)))
            return True

    def shutdown(self):

        rospy.loginfo("Stopping the robot...")
        if(self._tbasm.is_active()):
            self.cmd_vel.publish(Twist())
            self._as.set_aborted(self._result)
            rospy.sleep(0.1)


# keep it here for a rotation server
#    def quaternion_to_theta(self, odom):
#        t1 = +2.0 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y)
#        t2 = +1.0 - 2.0 * (odom.pose.pose.orientation.y ** 2 + odom.pose.pose.orientation.z**2)
#        return math.degrees(math.atan2(t1, t2))


if __name__ == '__main__':
    MovetodistanceServer()          
    rospy.spin()