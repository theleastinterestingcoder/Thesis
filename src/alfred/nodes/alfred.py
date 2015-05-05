#!/usr/bin/env python

"""
   alfred.py

   Written by Quan Zhou on 3/13

   Sends navigation goals based on key words. When it reaches goal, face recognition software
     is launched. 
"""
# standard pythonic libraries
import os, sys, subprocess, signal, time, pdb

# ROS imports
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Some of my own packages
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "components"))
from nav_goal_manager import nav_goal_manager
from face_recognition_spawner import face_recognition_spawner
from kobuki_sound_manager import kobuki_sound_manager
from raw_vel_commander import raw_vel_commander

# Import auxiliary functions
from mission_node import node
from mission_manager import mission_manager

# For colorful output
from termcolor import colored

class alfred:
    loc = {}
    loc['alpha'] = [3, 2, 0]
    loc['beta']  = [4, 4, 0]
    loc['home']  = [2.059158, -0.0460, 1.981]

    def __init__(self, name = 'alfred'):
        # Initialize this node if it has not been initialized already
        if not rospy.core.is_initialized():
            rospy.init_node(name)

        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speechCb)

        # Get copies of default locations of alpha and beta
        self.loc = alfred.loc
        self.pause_speech = False

        # Functional Modules
        self.ngm = nav_goal_manager()
        self.fds = face_recognition_spawner()
        self.ksm = kobuki_sound_manager()

        # Setup a publisher for simple navigation commander (note: raw_vel_cmd.py must be running)
        self.rvc_pub = rospy.Publisher('/alfred/raw_vel_commander/', String, queue_size=1)

        # Auxiluary Modules
        self.mission_manager = mission_manager(core_module=self)
        
        
        # Setup the resouces for missions
        self.missions = []   # A list of threads
        self.resources = {}  # A dictionary of arguments

        # A mapping from keywords to commands.
        self.keywords = {'ngm' : ['go to alpha', 'go to beta', 'go home', 'abort goals'], 
                         'rvc' : ['move foward', 'move right', 'turn left', 'turn right', 'stop', 'stop broadcast', 'start broadcast'],
                         'aux' : ['cancel', 'set mark alpha', 'set mark beta', 'pause speech', 'continue speech'],
                         'frs' : ['start face recognition', 'stop face recognition'],
                         'eliza' : ['start psychotherapist']}
        # Map actions
        self.keyword_to_node = {
            'go to alpha' :   node(function=self.core_component.ngm.go_to_location, *self.core_component.loc['alpha']),
            'go to beta' :    node(function=self.core_component.ngm.go_to_location, *self.core_component.loc['beta']), 
            'go to home' :    node(function=self.core_component.ngm.go_to_location, *self.core_component.loc['home']),
            'go home' :       node(function=self.core_component.ngm.go_to_location, *self.core_component.loc['home']),
            'move foward':    node(function=self.rvc_pub.publish,  *['move_foward']), 
            'move right':     node(function=self.rvc_pub.publish,  *['turn_right']), 
            'turn left':      node(function=self.rvc_pub.publish,  *['turn_left']),
            'turn right':     node(function=self.rvc_pub.publish,  *['turn_right']), 
            'stop':           node(function=self.rvc_pub.publish,  *['stop']), 
            'stop broadcast': node(function=self.rvc_pub.publish,  *['stop_broadcast']), 
            'start broadcast':node(function=self.rvc_pub.publish,  *['start_broadcast']), 
            'cancel' :        node(function=self.mission_manager.reset), 
            'set mark alpha' : None, 
            'set mark beta' : None, 
            'pause speech' : None, 
            'continue speech' : None, 
            'start face recognition' : None, 
            'stop face recognition' : None, 
            'start psychotherapist' : None, 
            'success beep':   node(function=self.core_component.ksm.beep, val = 1, done_cb=None),  
            'fail beep':      node(function=self.core_component.ksm.beep, val = 2, done_cb=None), 
            'go to location': node(function=self.core_component.ngm.go_to_location), 
        } 

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

        # # Goes to the goal and then beeps
        # success_nd =   node(function=self.ksm.beep, **{'val': 1, 'done_cb': None})
        # fail_nd =   node(function=self.ksm.beep, **{'val': 2, 'done_cb': None})
        # secondary = node(function=self.ngm.go_to_location, *[2,2] , success_nd = success_nd, fail_nd = fail_nd)

        # home_nd = node(function=self.ngm.go_to_location, *self.loc['home'], success_nd = success_nd, fail_nd = fail_nd)
        
        # timeout = {}
        # timeout['time'] = 5
        # timeout['mission'] = {'name': 'timeout go home', 'start_node':home_nd, 'do_now':True}


        # # An example of chaining (note, has to go backwards b/c of the first needs to reference the next)
        # fail_nd = node(function=self.ksm.beep, val = 2)
        # success_nd = node(function=self.ksm.beep, val=1)

        # # Manages which module gets to send messages to raw_vel_cmd 
        # if command in self.keywords['ngm']:
        #     self.rvc_pub.publish('stop broadcast')
        # elif command in self.keywords['rvc']:
        #     self.ngm.cancel_goals()

        # # Symbol -> Semantics; Syntax is almost a little bit too expresive, but understandable
        # #   once you pick it apart
        # if command == 'cancel' or command == 'abort goals':
        #     self.mission_manager.reset()
        # elif command == 'go to alpha':
        #     mission_f = node(function=self.ngm.go_to_location, *self.loc['alpha'], success_nd=success_nd, fail_nd=fail_nd)
        # elif command == 'go to beta':
        #     mission_f = node(function=self.ngm.go_to_location, *self.loc['beta'], success_nd=success_nd, fail_nd=fail_nd)
        # elif command == 'go home':
        #     mission_f = node(function=self.ngm.go_to_location, *self.loc['home'], success_nd=success_nd, fail_nd=fail_nd)
        # elif command == 'set mark alpha':
        #     self.loc['alpha'] = self.ngm.get_current_position()
        # elif command == 'set mark beta':
        #     self.loc['beta'] = self.ngm.get_current_position()
        # elif command in self.keywords['rvc']:
        #     self.rvc_pub.publish(command)

        mission_f = self.keyword_to_node[command]
        # If a mission has been formed, then execute the thread
        if 'mission_f' in locals():
            self.mission_manager.handle_request(command, mission_f, timeout=timeout)
            self.mission_manager.start()

        print colored('>', 'green'),

        return
            
    def cleanup(self):
        # When shutting down be sure to stop the robot! 
        self.ngm.cancel_goals()
        self.rvc_pub.publish('stop')

    # Resets the resources available to the robot
    def reset(self):
        self.ngm.cancel_goals()
        self.rvc_pub.publish('stop')

if __name__=="__main__":
    alfred()
    

