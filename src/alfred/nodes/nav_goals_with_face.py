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
from nav_goal_manager import nav_goal_manager
from face_recognition_spawner import face_recognition_spawner
from kobuki_sound_manager import kobuki_sound_manager

import os, subprocess, time, signal

# A callback function
from callback_function import cb_func

class voice_cmd:
    loc = {}
    loc['alpha'] = [3, 2, 0]
    loc['beta']  = [4, 4, 0]
    loc['home']  = [1.178, -0.43365, 0.010]

    def __init__(self, name = 'voice_cmd'):
        # Initialize this node if it has not been initialized already
        if not rospy.core.is_initialized():
            rospy.init_node(name)
        # Initialize the Twist message we will publish.
        self.msg = Twist()

        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speechCb)
        
        # A mapping from keywords to commands.
        self.keywords_to_command = { 'cancel' : 'cancel', 
                                     'go to alpha': 'go to alpha',
                                     'go to beta' : 'go to beta',
                                     'go home' : 'go home', 
                                     'set mark alpha' : 'set mark alpha',
                                     'set mark beta' : 'set mark beta',
                                     }
#                                      'move foward' : 'move foward',
#                                      'turn left' : 'turn left',
#                                      'turn right' : 'turn right',
#                                      'stop' : 'stop',
#                                     }
        # Get copies of default locations of alpha and beta
        self.loc = voice_cmd.loc

        # Core processes
        self.ngm = nav_goal_manager()
        self.fds = face_recognition_spawner()
        self.ksm = kobuki_sound_manager()
#         self.snc = subprocess.Popen("rosrun alfred simple_nav_commander.py", stdout=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        
        # Setup a publisher for simple navigation commander
#         self.snc_pub = rospy.Publisher('/alfred/simple_nav_commander/', String, queue_size=1)

        # Some other stuff
        rospy.loginfo("Ready to receive voice commands")
        rospy.on_shutdown(self.cleanup)

        rospy.spin()

    def get_command(self, data):
        # Convert a string into a command
        ans =  self.keywords_to_command.get(data)
        if not ans:
            rospy.loginfo('Warning: command not recognized "%s"' % data)
        return ans
#         for (command, keywords) in self.keywords_to_command.iteritems():
#             for word in keywords:
#                 if data.find(word) > -1:
#                     return command
        
    def speechCb(self, msg):        
        # Triggers on messages to /recognizer/output
        command = self.get_command(msg.data)
        rospy.loginfo("Command: " + str(command))

        # Goes to the goal and then beeps
#         done_cb =   cb_func(function=self.ksm.beep, **{'val': 1, 'done_cb': None})
        
        # An example of chaining (note, has to go backwards b/c of the first needs to reference the next)
#         finish_cb = cb_func(function=self.ksm.beep, val= 1, done_cb=None)
#         return_cb = cb_func(function=self.ngm.go_to_location, *self.loc['home'], done_cb=finish_cb)
#         face_cb   = cb_func(function=self.fds.look_for_face, names=['Quan'], duration=10, done_cb = return_cb)
#         done_cb   = cb_func(function=self.ksm.beep, val= 1, done_cb=face_cb)
        fail_cb = cb_func(function=self.ksm.beep, val = 2, done_cb=None)
        finish_cb = cb_func(function=self.ksm.beep, val= 1, success_cb = None)
        return_cb = cb_func(function=self.ngm.go_to_location, *self.loc['home'], success_cb = finish_cb)
        face_cb   = cb_func(function=self.fds.look_for_face, names=['Quan'], duration=10, success_cb = return_cb, fail_cb=fail_cb)
        done_cb = cb_func(function=self.ksm.beep, val=1, success_cb=face_cb)

#         done_cb = cb_func(**{'cb_f': self.look_for_face, 'name': ['Quan'], 'duration': 10})
#         done_cb = cb_func(**{'cb_f': self.look_for_face, 'name': ['Quan'], 'duration': 10})
#         done_cb = cb_func(**{'cb_f': self.look_for_face, 'name' : ['Quan'], 'duration' : 10})
#         return_cb = cb_func(**{'cb_f': self.go_to_location}, *self.loc['alpha']}) 

        if command == 'cancel':
            self.ngm.cancel_all_goals()
        elif command == 'go to alpha':
            self.ngm.go_to_location(*self.loc['alpha'], done_cb=done_cb)
        elif command == 'go to beta':
            self.ngm.go_to_location(*self.loc['beta'], done_cb=done_cb)
        elif command == 'go home':
            self.ngm.go_to_location(*self.loc['home'], done_cb=None)
        elif command == 'set mark alpha':
            self.loc['alpha'] = self.ngm.get_current_position()
        elif command == 'set mark beta':
            self.loc['beta'] = self.ngm.get_current_position()
#         elif command == 'move foward':
#             self.snc_pub.publish('move foward')
#         elif command == 'move backward':
#             self.snc_pub.publish('move backward')
#         elif command == 'turn left' : 
#             self.snc_pub.publish('turn left')
#         elif command == 'turn right':
#             self.snc_pub.publish('turn right')
#         elif command == 'stop':
#             self.snc_pub.publish('stop')

        return
        
            
    def print_greeting(self, greeting='foo'):
        print "I think this works! %s" % greeting
       
    def cleanup(self):
        # When shutting down be sure to stop the robot! 
        self.ngm.cancel_all_goals()
#         try:
#             os.killpg(self.snc, signal.SIGTERM)
#         except:
#             rospy.loginfo('Error, Could not kill simple_nav_commander for some reason')
  

          

if __name__=="__main__":
    voice_cmd()
    

