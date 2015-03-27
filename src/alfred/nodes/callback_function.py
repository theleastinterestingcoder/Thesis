'''
  callback_function.py
  
  written by Quan Zhou on 3/19/15
  Edited on 3/25/15

  A wrapper class that is a special callback function with parameters
'''

import pdb
import rospy
import actionlib

class cb_func:
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
        if args and args[0] == 3:
            ans =  self.function(*self.p_args, **self.p_kwargs)
        else:
            return False

        # Special case for server based actions
        if isinstance(ans, actionlib.simple_action_client.SimpleActionClient):
            pass
        if self.pd_cb:
            return self.pd_cb.call_back()
        elif ans and self.ps_cb:
            return self.ps_cb.call_back()
        elif self.pf_cb:
            return self.pf_cb.call_back()

    def __repr__(self):
        return "{function: %s\nargs: %s\n kwargs: %s}\n" % (self.function, self.p_args, self.p_kwargs)

