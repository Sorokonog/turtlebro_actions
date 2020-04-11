import rospy

from turtlebro_actions.msg import movetopointGoal, movetopointResult, movetopointFeedback  
from turtlebro_actions.msg import movetimeGoal, movetimeResult, movetimeFeedback
from turtlebro_actions.msg import movetodistanceGoal, movetodistanceResult, movetodistanceFeedback

dgoal = movetodistanceGoal()
dresult = movetodistanceResult()
dfeedback = movetodistanceFeedback()

tgoal = movetimeGoal()
tresult = movetimeResult()
tfeedback = movetimeFeedback()

cgoal = movetopointGoal()
cresult = movetopointResult()
cfeedback = movetopointFeedback()

print (dgoal)
print (dresult)
print (dfeedback)
print('==================')
print (tgoal)
print (tresult)
print (tfeedback)
print('==================')
print (cgoal)
print (cresult)
print (cfeedback)
