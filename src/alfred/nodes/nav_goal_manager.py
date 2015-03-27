#!/usr/bin/env python

'''
     navigation_stack.py 

     Controls the mobile base using simple speech commands. 

'''
import roslib; #roslib.load_manifest('robot_red')
import rospy
import actionlib
import pdb, time

#move_base_msgs
from move_base_msgs.msg import *

# Transformations
from tf import transformations, TransformListener
from termcolor import colored

# For extended callback functions


# Create a node and manages goals
class nav_goal_manager:
    # This is kinda hacky, but every nav_goal_manager will share the same tfl coordinates
    # Note: this might break if there are multiple nav_goal_managers

    def __init__(self):
        # Initialize some basic stuff here
        self.sac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.marks = []                           # User defined locations
        self.tfl = TransformListener()

        self.cbf = {}                             # Dictionary of Callback Functions
        self.cba = {}                             # Dictionary of Callback Parameters

    def add_mark(self, x=0, y=0, z=0, w=0):
        # Addes a mark to user defined coordinates
        self.marks.append((x, y, z, w))
    
    def set_mark(self, mark_num, x=0, y=0, z=0, w=0):
        # changes the mark
        self.marks[mark_num] = (x, y, z, w)
    
    def remove_mark(self, mark_num):
        # Removes a mark
        self.marks[mark_num] = (None, None, None, None)

    def go_to_mark(self, mark_num):
        # Goes to the mark, if it is a valid mark
        # Do some validation on the data
        if mark_num > len(self.marks):
            rospy.loginfo(colored('Error: mark_num=%s is out of bounds (self.marks =  %s)' % (mark_num, self.marks), 'red'))
            return
        elif self.marks[mark_num][0] == None:
            rospy.loginfo(colored('Error: mark_num=%s has been removed (self.marks[%s] = %s)' (mark_num, mark_num, self.marks[mark_num])), 'red')
            return
        else:
            self.go_to_location(self.marks[mark_num])
            rospy.loginfo('Sending Goal with mark_num=%s with (x,y,z,w) = %s' % (mark_num, self.marks[mark_num]))

    def go_to_location(self, x=0, y=0, z=0, w=0, done_cb=None):
        # Wrapper for go_to_goal
        goal = form_goal(x,y,z,w)
        return self.go_to_goal(goal, done_cb=done_cb)

    def go_to_goal(self, goal, done_cb=None):
        # Wait for the server, send the goal, and then publish the result
        self.sac.wait_for_server()
        loc_x = goal.target_pose.pose.position.x
        loc_y = goal.target_pose.pose.position.y
        loc_z = goal.target_pose.pose.position.z

        rospy.loginfo('Sending goal with (x,y,z) =[%s, %s, %s]' % (loc_x, loc_y, loc_z))

        self.sac.send_goal(goal)
        ans = self.sac.wait_for_result()

        return ans

    def done_goal_callback(self, a,b):
        rospy.loginfo('At position: (x,y,z,w) =  (%s, %s, %s, %s)' % self.get_current_position())

    def get_current_position(self):
        # Returns [x, y, w], where (x,y) are locations and w is the yaw
        base_f = rospy.get_param('~base_frame', '/base_link')                                                          
        map_f  = rospy.get_param('~map_frame', '/map')

        now = rospy.Time(0)
        self.tfl.waitForTransform(map_f, base_f, now, rospy.Duration(4.0))
        (pos,quat) = self.tfl.lookupTransform(map_f, base_f, now)
        euler = transformations.euler_from_quaternion(quat)
        return (pos[0], pos[1], pos[2], euler[2])

    def cancel_all_goals(self):
        self.sac.cancel_all_goals()


def form_goal(x=0, y=0, z=0, w=0):
    # Create a goal and return it
    goal = MoveBaseGoal()
    
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    
    quat = transformations.quaternion_from_euler(0,0,w)
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    return goal



if __name__ == '__main__':
    from callback_function import cb_func
    try:
        rospy.init_node('nav_goal_manager')
        ngm = nav_goal_manager()
        
        # When the goal success, call the callback function
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
