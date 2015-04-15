'''
  mission_node.py
  
  written by Quan Zhou on 3/19/15
  Edited on 3/25/15
  Edited on 4/15/15

  A node in a mission DFA. Contains:
  - a constructor with arguments:
        'function' : function_pointer
        'success_nd' : pointer to node to call upon success
        'fail_nd' : pointer to call upon failure
'''

import rospy, actionlib

class node:
    # First strip special arguments, then deliver rest as payload
    def __init__(self, *args, **kwargs):
        self.function = kwargs.pop('function')
        self.ps_nd = kwargs.pop('success_nd', None)
        self.pf_nd = kwargs.pop('fail_nd', None)

        self.p_kwargs = kwargs
        self.p_args = args
    
    # Call the function, and then depending on the results, call the successor function
    def execute(self, *args, **kwargs):
        ans =  self.function(*self.p_args, **self.p_kwargs)
        
        # None is a signal for a Canceled function
        if ans == None:
            rospy.loginfo('Mission_node: Mission Ended with an canceled Action')
            return False
        
        # Call the successful action
        if ans and self.ps_nd:
            return self.ps_nd.execute()
        elif self.pf_nd:
            return self.pf_nd.execute()

    def __repr__(self):
        return "{function: %s\nargs: %s\n kwargs: %s}\n" % (self.function, self.p_args, self.p_kwargs)

