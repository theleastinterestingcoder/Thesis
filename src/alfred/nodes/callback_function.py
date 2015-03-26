'''
  callback_function.py
  
  written by Quan Zhou on 3/19/15
  Edited on 3/25/15

  A wrapper class that is a special callback function with parameters
'''

import pdb
import rospy
from nav_goal_manager import nav_goal_manager as ngm

class cb_func:
    action_lib_func = [ngm.go_to_location, ngm.go_to_goal]

    # First strip special arguments, then deliver rest as payload
    def __init__(self, *args, **kwargs):
        self.function = kwargs.pop('function')
        self.ps_cb = kwargs.pop('success_cb', None)
        self.pf_cb = kwargs.pop('fail_cb', None)
        self.pd_cb = kwargs.pop('done_cb', None)

        self.p_kwargs = kwargs
        self.p_args = args
    
    # Call the function, and then depending on the results, call the successor function
    def call_back(self, *args, **kwargs):

        ans =  self.function(*self.p_args, **self.p_kwargs)
        # Special case for server based actions
        if ans in cb_func.action_lib_func:
            ans.wait_for_result()
        if self.pd_cb:
            return self.pd_cb.call_back()
        elif ans and self.ps_cb:
            return self.ps_cb.call_back()
        elif self.pf_cb:
            return self.pf_cb.call_back()

    def __repr__(self):
        return "{function: %s\nargs: %s\n kwargs: %s}\n" % (self.function, self.p_args, self.p_kwargs)

