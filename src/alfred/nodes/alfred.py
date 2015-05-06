#!/usr/bin/env python

"""
   alfred.py

   Written by Quan Zhou on 3/13

   Sends navigation goals based on key words. When it reaches goal, face recognition software
     is launched. 
"""
# standard pythonic libraries
import os, sys, subprocess, signal, time, pdb, re

# ROS imports
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Some of my own packages
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "components"))
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "components/middle_managers"))
from nav_goal_manager import nav_goal_manager
from face_recognition_spawner import face_recognition_spawner
from kobuki_sound_manager import kobuki_sound_manager
from raw_velocity_client import raw_velocity_client
from middle_manager_coordinator import coordinator
from verbal_tokenizer import verbal_tokenizer

# Import auxiliary functions
from mission_node import node
from mission_manager import mission_manager

# For colorful output
from termcolor import colored

# For semantic parsing
from semantic import numbers
ns = numbers.NumberService()

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
        self.coordinator = coordinator(self)

        self.ngm = nav_goal_manager()
        self.frs = face_recognition_spawner(self.coordinator)
        self.ksm = kobuki_sound_manager()
        self.rvc = raw_velocity_client(self.coordinator)

        # Auxiluary Modules
        self.mission_manager = mission_manager(coordinator=self.coordinator)
        
        
        # Setup the resouces for missions
        self.missions = []   # A list of threads
        self.resources = {
            'alpha' : alfred.loc['alpha'],
            'beta'  : alfred.loc['beta'],
            'home'  : alfred.loc['home'],
        }  # A dictionary of arguments
        
        # Setup the standard set of nodes
        success_beep = node(function=self.ksm.beep, *[1] )
        fail_beep    = node(function=self.ksm.beep, *[2] )  

        time_expression = r'(.+(?:' + '|'.join(verbal_tokenizer.time_words.keys()) + r'))';
        self.keyword_to_node = {
            # 'cancel' :        node(function=self.mission_manager.reset), 

            'control' : {
                r'cancel':         node(function=self.coordinator.cancel, success_nd=fail_beep),
                r'stop':           node(function=self.coordinator.cancel, success_nd=fail_beep),
            },

            'raw_velocity_client': {
                r'move fowards':   node(function=self.rvc.move_foward), 
                r'move foward':    node(function=self.rvc.move_foward), 
                r'move backwards': node(function=self.rvc.move_backward), 
                r'move backward':  node(function=self.rvc.move_backward),
                r'move right':     node(function=self.rvc.turn_right), 
                r'turn left':      node(function=self.rvc.turn_left),
                r'turn right':     node(function=self.rvc.turn_right), 
                r'stop':           node(function=self.rvc.stop), 
            },

            'navigation_goal_manager': {
                r'go to alpha' :   node(function=self.ngm.go_to_location, *self.resources['alpha'], success_nd=success_beep, fail_nd=fail_beep),
                r'go to beta' :    node(function=self.ngm.go_to_location, *self.resources['beta'], success_nd=success_beep, fail_nd=fail_beep), 
                r'go to home' :    node(function=self.ngm.go_to_location, *self.resources['home'], success_nd=success_beep, fail_nd=fail_beep),
                r'go home' :       node(function=self.ngm.go_to_location, *self.resources['home'], success_nd=success_beep, fail_nd=fail_beep),
                r'get current location' : node(function=self.ngm.get_current_position),
            },
            
            'kobuki_sound_manager' : {
                r'fail beep'   : fail_beep,
                r'success beep': success_beep,
                r'beep with value (\w+)' : node(function=self.ksm.beep, *[4]),
            },

            'aux' : {
                # wait for <time val> 
                r'wait ' + time_expression  : node(function=rospy.sleep, *[5], success_nd=success_beep, fail_nd=fail_beep),
                r'set mark alpha' : node(function=self.set_mark, target='alpha'),
                r'set mark beta'  : node(function=self.set_mark, target='beta'), 
                r'set resource (\w+) as (\w+)' : node(function=self.save_keyword_variable),
                r'check resource (\w+) as (\w+)' : node(function=self.check_keyword_variable),

            },

            'face_recognition_spawner' : {
                r'look for (\w+)':  node(function=self.frs.look_for_face, success_nd=success_beep, fail_nd=fail_beep),
            }
        }



            
            # 'set mark alpha' : None, 
            # 'set mark beta' : None, 
            # 'pause speech' : None, 
            # 'continue speech' : None, 
            # 'start face recognition' : None, 
            # 'stop face recognition' : None, 
            # 'start psychotherapist' : None, 
            # 'success beep':   node(function=self.ksm.beep, val = 1, done_cb=None),  
            # 'fail beep':      node(function=self.ksm.beep, val = 2, done_cb=None), 
            # 'go to location': node(function=self.ngm.go_to_location), 
        

        # For parsing commands
        self.vt = verbal_tokenizer(self)

        self.keyword_groups = {
            r'beep with value (\w+)': self.vt.parse_numeric,
            r'wait for '+ time_expression : self.vt.parse_time, 
            r'look for (\w+)' : self.vt.parse_string, 
            r'set resource (\w+) as (\w+)' : self.vt.parse_tuple,
            r'check resource (\w+) as (\w+)' : self.vt.parse_tuple,
        }
        
        # ---- Wrap up this long constructor -----
        time.sleep(0.1)
        rospy.loginfo("Ready to receive voice commands")
        rospy.on_shutdown(self.coordinator.cleanup)
        rospy.spin()

    def parse_node_from_command_token(self, token):
        # Convert a token into a node
        module, command, node = self.get_items_from_keyword_commands(token)

        # Deal with capture groups, if need be
        if re.search(r'\(.*\)', command):
            groups = re.search(command, token).groups()
            
            # look for the proper capture group
            action = self.keyword_groups[command]
            if type(groups) == tuple and action != self.vt.parse_tuple:
                groups = groups[0]

            p_args = action(groups)

            if action == self.vt.parse_tuple:
                node.p_args = p_args
            else:
                node.p_args = (p_args,)
                

        return node

    def get_items_from_keyword_commands(self, token):
        for tyype, mapping in self.keyword_to_node.iteritems():
            for keyword in mapping.keys():
                reg = re.compile(r'\b'+keyword+r'\b')
                if reg.search(token):
                    return tyype, keyword, mapping[keyword]

        raise Exception('Warning: command token not recognized "%s"' % token)

                
    def parse_environment_from_modifier(self, modifier):
        ans = {'do_now' : True, 'timeout' : None}

        # No modifiers
        if not modifier:
            return ans

        if 'then' in modifier:
            ans['do_now'] = False
        if 'for' in modifier:
            tm = self.vt.parse_time(modifier.replace('for', '').replace('now', '').replace('then', ''))
            ans['timeout'] = {
                'time' : tm,
                'mission' : {
                    'name' : 'modifier timeout',
                    'start_node': self.keyword_to_node['kobuki_sound_manager']['fail beep'], 
                    'do_now':  True,
                }
            }
        return ans
        

    def speechCb(self, msg):        
        # Triggers on messages to /recognizer/output
        tokens, color = self.vt.tokenize_string(msg.data)

        # For each token, do a request
        for t,c in zip(tokens, color):
            rospy.loginfo('handing token %s as %s' % (t,c))
            modifier = self.parse_environment_from_modifier(t[0])
            module, command, foo = self.get_items_from_keyword_commands(t[1])
            node = self.parse_node_from_command_token(t[1])

            self.handle_command(modifier, node, module, command)

    def handle_command(self, modifier, node, module, command):
        if module == None:
            return
        mission_t = self.keyword_to_node[module][command]
        # If it is a control keyword, then execute it now
        if module == 'control':
            mission_t.execute()
            return
        if module == 'control' and modifier['do_now'] == False:
            raise Exception('Error: control-typed command "%s" placed in a chain of verbal commands' % command)
        if command in ['go to alpha', 'go to beta', 'go home', 'go to home']:
            node.p_args = self.resources[command.replace('go','').replace('to','').strip()]
            mission_t = self.keyword_to_node[module][command]
        if modifier['do_now']:
            self.coordinator.cancel()
            self.mission_manager.clear_mission_queue()

        rospy.loginfo("--- Command: %s ---" % str(command))
        # If a mission has been formed, then execute the thread
        self.mission_manager.handle_request(command, mission_t, timeout=modifier['timeout'])

    # --- some of our own functions ----
    def save_keyword_variable(self, key, val):
        rospy.loginfo('Saving "%s"="%s"' % (key,val))
        self.resources[key] = val
        return True

    def check_keyword_variable(self, key, val):
        if key not in self.resources:
            return False
        ans = self.resources[key] == val
        rospy.loginfo('aux: resources[%s] == "%s" evaluates to %s' % (key, val, ans))
        return ans

    def set_mark(self, target):
        self.resources[target] = self.ngm.get_current_position()
        rospy.loginfo(str(self.resources))
        return True

            
    def cleanup(self):
        # When shutting down be sure to stop the robot! 
        self.reset()

    # Resets the resources available to the robot
    def reset(self):
        self.ngm.cancel_goals()
        self.rvc.stop()

if __name__=="__main__":
    alfred()
    

