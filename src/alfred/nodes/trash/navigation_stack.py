#!/usr/bin/env python

'''
     navigation_stack.py 

     Controls the mobile base using simple speech commands. 

'''
import roslib; #roslib.load_manifest('robot_red')
import rospy
import actionlib
import pdb

#move_base_msgs
from move_base_msgs.msg import *

locations = []

# Create a node and manages goals
class NavGoalManager:
    def __init__(self, name='nav_goal_manager'):
        rospy.init_node(name)
        self.sac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def go_to_goal(self, goal):
        # Wait for the server, send the goal, and then publish the result
        self.wait_for_server()
        loc_x = goal.target_pose.pose.position.x
        loc_y = goal.target_pose.pose.position.y
        loc_z = goal.target_pose.pose.position.z

        rospy.loginfo('Sending goal with loc=[%s, %s, %s]') % (loc_x, loc_y, loc_z)
        self.sac.send_goal(goal)

        # wait for process to finish
        res = self.sac.wait_for_result()

        # Print the result
        rospy.loginfo('Goal has: %s' % self.sac.get_result())

    def go_to_location(self, x=0, y=0, z=0, ww = 0, wx = 0, wy = 0, wz = 0):
        self.execute_goal(form_goal(x,y,z,ww,wx,wy,wz))

    def cancel_all_goals(self):
        self.sac.cancel_all_goals()

def form_goal(x=0, y=0, z=0, ww=0, wx=0, wy = 0, wz = 0):
    # Create a goal and return it
    goal = MoveBaseGoal()
    
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.x = y
    goal.target_pose.pose.position.y = z

    goal.target_pose.pose.orientation.w = ww
    goal.target_pose.pose.orientation.x = wx
    goal.target_pose.pose.orientation.y = wy
    goal.target_pose.pose.orientation.z = wz

    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    return goal

if __name__ == '__main__':
    try:
        ngm = NavGoalManager('nav_goal_manager')
        ngm.go_to_location(3,2)
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
