#!/usr/bin/env python

"""
   nav_goals_with_face.py

   Written by Quan Zhou on 3/13

   Sends navigation goals based on key words. When it reaches goal, face recognition software
     is launched. 
"""

import roslib; roslib.load_manifest('pi_speech_tutorial')
import rospy
import pdb, time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign
from termcolor import colored

# Some of my own packages
from nav_goal_manager import nav_goal_manager
from face_recognition_spawner import face_recognition_spawner
from kobuki_sound_manager import kobuki_sound_manager
from simple_nav_commander import simple_nav_commander

import os, subprocess, time, signal

# Import auxiliary functions
from callback_function import cb_func
from mission_thread import mission_thread

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
        self.keywords = {'ngm' : ['go to alpha', 'go to beta', 'go home', 'abort goals'], 
                         'rvc' : ['move foward', 'move right', 'turn left', 'turn right', 'stop', 'stop broadcast', 'start broadcast'],
                         'aux' : ['cancel', 'set mark alpha', 'set mark beta']}

        # Get copies of default locations of alpha and beta
        self.loc = voice_cmd.loc

        # Core processes
        self.ngm = nav_goal_manager()
        self.fds = face_recognition_spawner()
        self.ksm = kobuki_sound_manager()
#         self.rvc = subprocess.Popen("rosrun alfred simple_nav_commander.py", stdout=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        
        # Setup a publisher for simple navigation commander (note: raw_vel_cmd.py must be running)
        self.rvc_pub = rospy.Publisher('/alfred/raw_vel_commander/', String, queue_size=1)

        # Some other stuff
        time.sleep(0.1)
        rospy.loginfo("Ready to receive voice commands")
        rospy.on_shutdown(self.cleanup)

        rospy.spin()

    def get_command(self, data):
        # Convert a string into a command
        for module, commands in self.keywords.iteritems():
            if data in commands:
                return data
        rospy.loginfo('Warning: command not recognized "%s"' % data)
        return None
        
    def speechCb(self, msg):        
        # Triggers on messages to /recognizer/output
        command = self.get_command(msg.data)
        rospy.loginfo("Command: " + str(command))

        # Goes to the goal and then beeps
        success_cb =   cb_func(function=self.ksm.beep, **{'val': 1, 'done_cb': None})
        fail_cb =   cb_func(function=self.ksm.beep, **{'val': 2, 'done_cb': None})
        secondary = cb_func(function=self.ngm.go_to_location, *[2,2] , success_cb = success_cb, fail_cb = fail_cb)
        # An example of chaining (note, has to go backwards b/c of the first needs to reference the next)
#         fail_cb = cb_func(function=self.ksm.beep, val = 2, done_cb=None)
#         finish_cb = cb_func(function=self.ksm.beep, val= 1, done_cb= None)
#         return_cb = cb_func(function=self.ngm.go_to_location, *self.loc['home'], success_cb = finish_cb, fail_cb = fail_cb)
#         face_cb   = cb_func(function=self.fds.look_for_face, names=['Quan'], duration=10, success_cb = return_cb, fail_cb=fail_cb)
#         done_cb = cb_func(function=self.ksm.beep, val=1, success_cb=face_cb)

#         done_cb = cb_func(**{'cb_f': self.look_for_face, 'name': ['Quan'], 'duration': 10})
#         done_cb = cb_func(**{'cb_f': self.look_for_face, 'name': ['Quan'], 'duration': 10})
#         done_cb = cb_func(**{'cb_f': self.look_for_face, 'name' : ['Quan'], 'duration' : 10})
#         return_cb = cb_func(**{'cb_f': self.go_to_location}, *self.loc['alpha']}) 
        
        # Manages which module gets to send messages to raw_vel_cmd 
        if command in self.keywords['ngm']:
            self.rvc_pub.publish('stop broadcast')
        elif command in self.keywords['rvc']:
            self.ngm.cancel_all_goals()
        # Symbol -> Semantics; Syntax is almost a little bit too expresive, but understandable
        #   once you pick it apart
        if command == 'cancel' or command == 'abort goals':
            self.ngm.cancel_all_goals()
            self.rvc_pub.publish('stop')
        elif command == 'go to alpha':
            mission_f = cb_func(function=self.ngm.go_to_location, *self.loc['alpha'], success_cb=success_cb, fail_cb=fail_cb)
        elif command == 'go to beta':
            mission_f = cb_func(function=self.ngm.go_to_location, *self.loc['beta'], success_cb=success_cb, fail_cb=fail_cb)
        elif command == 'go home':
            mission_f = cb_func(function=self.ngm.go_to_location, *self.loc['home'], success_cb=success_cb, fail_cb=fail_cb)
        elif command == 'set mark alpha':
            self.loc['alpha'] = self.ngm.get_current_position()
        elif command == 'set mark beta':
            self.loc['beta'] = self.ngm.get_current_position()
        elif command in self.keywords['rvc']:
            self.rvc_pub.publish(command)
        
        # If a mission has been formed, then execute the thread
        if 'mission_f' in locals():
            mission_t = mission_thread(name=command, cb = mission_f) 
            mission_t.start()

        print colored('>', 'green'),

        return
        
            
    def print_greeting(self, greeting='foo'):
        print "I think this works! %s" % greeting
       
    def cleanup(self):
        # When shutting down be sure to stop the robot! 
        self.ngm.cancel_all_goals()
        try:
            os.killpg(self.rvc, signal.SIGTERM)
        except:
            rospy.loginfo('Error, Could not kill simple_nav_commander for some reason')
  

          

if __name__=="__main__":
    voice_cmd()
    

