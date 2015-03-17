#!/usr/bin/env python

"""
   nav_goals_with_face.py

   Written by Quan Zhou on 3/13

   Sends navigation goals based on key words. When it reaches goal, face recognition software
   is launched. 
"""

import roslib; roslib.load_manifest('pi_speech_tutorial')
import rospy
import pdb
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

from NavGoalManager import NavGoalManager

class voice_cmd_vel:
    loc_alpha = [3, 2, 0]
    loc_beta  = [4, 4, 0]

    def __init__(self, name = 'voice_cmd_vel'):
        # Initialize this node if it has not been initialized already
        if not rospy.core.is_initialized():
            rospy.init_node(name)
        # Initialize the Twist message we will publish.
        self.msg = Twist()

        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speechCb)
        
        # A mapping from keywords to commands.
        self.keywords_to_command = { 'cancel' : ['cancel'], 
                                     'go alpha': ['go to alpha'],
                                     'go beta' : ['go to beta'],
                                    }
        # Get copies of default locations of alpha and beta
        self.alpha = voice_cmd_vel.loc_alpha
        self.beta  = voice_cmd_vel.loc_beta

        self.ngm = NavGoalManager('nav_goal_manager')
        rospy.loginfo("Ready to receive voice commands")
        rospy.on_shutdown(self.cleanup)

        rospy.spin()

    def get_command(self, data):
        # Convert a string into a command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speechCb(self, msg):        
        # Triggers on messages to /recognizer/output
        command = self.get_command(msg.data)
        rospy.loginfo("Command: " + str(command))

        if command == 'cancel':
            self.ngm.cancel_all_goals()
        elif command == 'go alpha':
            self.ngm.go_to_location(*self.loc_alpha)
        elif command == 'go beta':
            self.ngm.go_to_location(*self.loc_beta)

        
    def cleanup(self):
        # When shutting down be sure to stop the robot! 
        self.ngm.cancel_all_goals()

if __name__=="__main__":

    voice_cmd_vel()
    

