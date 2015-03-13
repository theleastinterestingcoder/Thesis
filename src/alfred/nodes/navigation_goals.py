#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
from actionlib import SimpleActionClient
import rospy

from chores.msg import DoDishesAction, DoDishesGoal

from move_base_msgs import msg

class NavGoals():
    def __init__(self):
        # create a client
        self.ac = SimpleActionClient('simple_navigation_goals', msg.MoveBaseAction)
    
    def set_navigation_goal(self, loc, ori):
        #Sends a navigation goal to the stack and returns the state
        self.ac.wait_for_server()
        goal = make_navigation_goal(loc, ori)
        rospy.loginfo("Sending Pythonic Goal")
        self.ac.send_goal(goal)

        self.ac.wait_for_result()
        return client.getState()

def make_navigation_goal(loc, ori):
    # Create a navigation goal at location <loc> and orientation <ori>
    #Wait for the server to come up
    goal = msg.MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id = "base_link";
    goal.goal.target_pose.header.stamp = rospy.get_rostime()

    goal.goal.target_pose.pose.position.x = loc[0]
    goal.goal.target_pose.pose.position.y = loc[1]
    goal.goal.target_pose.pose.position.z = loc[2]

    goal.goal.target_pose.pose.orientation.w = ori[0]
    goal.goal.target_pose.pose.orientation.x = ori[1]
    goal.goal.target_pose.pose.orientation.y = ori[2]
    goal.goal.target_pose.pose_orientation.z = ori[3]
   
    return goal

    
