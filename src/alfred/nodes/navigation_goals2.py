#!/usr/bin/env python

import roslib; #roslib.load_manifest('robot_red')
import rospy
import actionlib
import pdb

#move_base_msgs
from move_base_msgs.msg import *

def simple_move():

    rospy.init_node('simple_move')
#    pdb.set_trace()
    #Simple Action Client
#     sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
    sac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #create goal
    goal = MoveBaseGoal()

    #use self?
    #set goal
    goal.target_pose.pose.position.x = 4.0
    goal.target_pose.pose.position.y = 4.0
    goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()

    #send goal
    sac.send_goal(goal)
    pdb.set_trace()
    #finish
    sac.wait_for_result()

    #print result
    print sac.get_result()


if __name__ == '__main__':
    try:
        simple_move()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
