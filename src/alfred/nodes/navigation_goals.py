#! /usr/bin/env python

import roslib
#roslib.load_manifest('alfred')
import rospy
from actionlib import SimpleActionClient
import rospy

from move_base_msgs import msg

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import *
from geometry_msgs.msg import *


rospy.init_node('hello_world')
rospy.loginfo('initialized node')

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x=-1.0
goal.target_pose.pose.orientation.w=-1.0
move_base.send_goal(goal)
rospy.spin()


# class NavGoals():
#     def __init__(self):
#         # create a client
#         self.ac = SimpleActionClient('simple_navigation_goals', msg.MoveBaseAction)
#     
#     def set_navigation_goal(self, loc, ori):
#         #Sends a navigation goal to the stack and returns the state
#         self.ac.wait_for_server()
#         goal = make_navigation_goal(loc, ori)
#         rospy.loginfo("Sending Pythonic Goal")
#         self.ac.send_goal(goal)
# 
#         self.ac.wait_for_result()
#         return self.ac.getState()
# 
# def make_navigation_goal(loc, ori):
#     # Create a navigation goal at location <loc> and orientation <ori>
#     #Wait for the server to come up
#     goal = msg.MoveBaseActionGoal()
# 
#     goal.goal.target_pose.header.frame_id = "base_link";
#     goal.goal.target_pose.header.stamp = rospy.get_rostime()
# 
#     goal.goal.target_pose.pose.position.x = loc[0]
#     goal.goal.target_pose.pose.position.y = loc[1]
#     goal.goal.target_pose.pose.position.z = loc[2]
# 
#     goal.goal.target_pose.pose.orientation.w = ori[0]
#     goal.goal.target_pose.pose.orientation.x = ori[1]
#     goal.goal.target_pose.pose.orientation.y = ori[2]
#     goal.goal.target_pose.pose.orientation.z = ori[3]
#    
#     return goal
# 
#     
