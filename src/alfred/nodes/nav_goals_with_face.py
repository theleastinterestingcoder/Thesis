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

# Some of my own packages
from NavGoalManager import NavGoalManager
from face_recognition_spawner import face_recognition_spawner
from kobuki_sound_manager import kobuki_sound_manager

import os, subprocess, time, signal

# A callback function
from callback_function import cb_func

class voice_cmd:
    loc = {}
    loc['alpha'] = [3, 2, 0]
    loc['beta']  = [4, 4, 0]
    loc['home']  = [1.929, -0.061, 0.010]

    def __init__(self, name = 'voice_cmd'):
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
        self.loc = voice_cmd.loc

        # Core processes
        self.ngm = NavGoalManager()
        self.fds = face_recognition_spawner()
        self.ksm = kobuki_sound_manager()

        # Some other stuff
        rospy.loginfo("Ready to receive voice commands")
        self.look_for_face()
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

#         done_cb = cb_func({'cb_f': self.look_for_face, 'name': ['Quan'], 'duration': 10})
#         done_cb = cb_func({'cb_f': self.look_for_face, 'name': ['Quan'], 'duration': 10})
        done_cb = cb_func(**{'function': self.ksm.beep, 'val': 1})

        if command == 'cancel':
            self.ngm.cancel_all_goals()
        elif command == 'go alpha':
            self.ngm.go_to_location(*self.loc['alpha'], done_cb=done_cb)
        elif command == 'go beta':
            self.ngm.go_to_location(*self.loc['beta'], done_cb=done_cb)
        elif command == 'go home':
            self.ngm.go_to_location(*self.loc['home'], done_cb=None)
            
    def print_greeting(self, greeting='foo'):
        print "I think this works! %s" % greeting

     
#    def look_for_face(names=['Quan'], duration=10, \
#                                      success_fp=self.go_to_location, success_arg=self.loc['home'], \
#                                      fail_fp=self.beep, fail_arg=1):

     # Look for a face and then do an action depending on sucess or failure

    def look_for_face(self, names = ['Quan'], duration=10):
        # Allow only face recognition function to be working at a time
        if not self.fds.is_busy:
            pdb.set_trace()
            self.fds.init_nodes() # Locks the fds
            self.fds.start_pub()
         
            # Check if a face has been seen
            if self.fds.watch_for(names, duration):
                self.go_to_location(*self.loc['home'])
                ans = True
            else:
                self.beep(1) 
                ans = False
          
            fds.stop_pub()
            fds.kill_everything() # Releases the lock
        else:
            rospy.info('Tried to look for face, but Face Detection Service is busy!')
 
#         start = rospy.Time.now();
#         
#         fp_success=self.ngm.go_to_location
#         fp_failure=rospy.loginfo
# 
#         self.launch_face_recognition(wait_time=10, success=fp_success, failure=fp_failure)
#         
#         while rospy.
#         # If you've seen Quan since the start of the navigation goal
#         if self.fds.seen('Quan', start):
#             self.ngm.go_to_location(*self.loc_home)
        
       
    def cleanup(self):
        # When shutting down be sure to stop the robot! 
        self.ngm.cancel_all_goals()
  
#     # Some scripting here to launch the face recognition stuff
#     def launch_face_recognition(self):
#         fds = face_recognition_spawner() 
#         fds.init_nodes()
#         fds.start_pub()
#         for i in range(10):
#             rospy.loginfo("Countdown to kill is %s" % i)
#             time.sleep(1)
#         fds.stop_pub()
#         fds.kill_everything()

          

if __name__=="__main__":
    voice_cmd()
    

