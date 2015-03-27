#! /usr/bin/env python
'''
    raw_cmd_listener.py

    Written by Quan Zhou on March 26, 2015

    Controls simple commands of the mobile base.
    Based on the voice_cmd_vel.py script by Michael Ferguson in the pocketsphinx ROS package.
'''

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

import os

class raw_cmd_listener:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.init_node('raw_cmd_manager')
        self.max_speed = rospy.get_param("~max_speed", 0.6)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed = rospy.get_param("~start_speed", 0.3)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)
        self.linear_increment = rospy.get_param("~linear_increment", 0.05)
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        self.paused = False
    
        # Initialize the Twist message we will publish.
        self.msg = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=1)

        # Subscribe to the command _velocity command
        self.cmd_sub  = rospy.Subscriber('/alfred/raw_cmd_manager/', String, self.handle_commands)

        self.cmd_map = {'move foward': self.move_foward,
                        'move backward' : self.move_backward,
                        'turn left' : self.turn_left,
                        'turn right': self.turn_right, 
                        'stop': self.stop}


        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.msg)
            r.sleep()   

    # catch commands from /alfred/raw_cmd_manager/ and publish them
    def handle_commands(self, data):
        func = self.cmd_map.get(data.data)
        if func:
            func()
        else:
            rospy.loginfo('raw_cmd_manager: Error, command not recognized "%s"' % data.data)


    def move_foward(self):
        self.msg.linear.x = self.speed
        self.msg.angular.z = 0
        self.remake_msg()

    def turn_left(self):
        if self.msg.linear.x != 0:
            self.msg.angular.z += self.angular_increment
        else:        
            self.msg.angular.z = self.angular_speed
        self.remake_msg()

    def turn_right(self):
        if self.msg.linear.x != 0:
            self.msg.angular.z -= self.angular_increment
        else:        
            self.msg.angular.z = -self.angular_speed
        self.remake_msg()

    def move_backward(self):    
        self.msg.linear.x = -self.speed
        self.msg.angular.z = 0
        self.remake_msg()

    def stop(self):
        self.msg = Twist()
        self.remake_msg()

    def remake_msg(self):
        self.msg.linear.x = min(self.max_speed, max(-self.max_speed, self.msg.linear.x))
        self.msg.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.msg.angular.z))

    def cleanup(self):
        # When shutting down be sure to stop the robot!  Publish a Twist message consisting of all zeros.
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

if __name__=="__main__":
    rospy.init_node('raw_cmd_manager')
    snc = raw_cmd_manager()
    # From the command prompt:
    # $ python raw_cmd_manager.py
    # $ rostopic pub -1 /alfred/raw_cmd_manager std_msgs/String -- "move foward"
    # $ rostopic pub -1 /alfred/raw_cmd_manager std_msgs/String -- "stop""
