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

import json, pdb
from types import FunctionType

# import rospy

class node:
    # First strip special arguments, then deliver rest as payload
    def __init__(self, *args, **kwargs):
        self.function = kwargs.pop('function')
        self.name = kwargs.pop('name', "")
        self.ps_nd = kwargs.pop('success_nd', None)
        self.pf_nd = kwargs.pop('fail_nd', None)

        # Get the arguments and keyword arguments for function
#         if 'p_args' in kwargs:
#             self.p_args = kwargs.pop('p_args')
#             self.p_args.extend(args)
#         else:
#             self.p_args = args
# 
#         if 'p_kwargs' in kwargs:
#             self.p_kwargs = kwargs.pop('p_kwargs')
#             self.p_kwargs.update(kwargs)
#         else:
#             self.p_kwargs = kwargs
        self.p_args = args
        self.p_kwargs = kwargs


    
    # Call the function, and then depending on the results, call the successor function
    def execute(self, *args, **kwargs):
        ans =  self.function(*self.p_args, **self.p_kwargs)
        # None is a signal for a Canceled function
        if ans == None:
            return None
        
        # Call the successful action
        if ans and self.ps_nd:
            return self.ps_nd.execute()
        elif self.pf_nd:
            return self.pf_nd.execute()

    def __repr__(self):
        # represent pointers to the next nodes
        if not self.ps_nd:
            success_func = None
        elif type(self.ps_nd) == str:
            success_func = 'String reference: "%s"' % self.ps_nd
        else:
            success_func = self.ps_nd.__dict__.get('function', None)

        # represent pointers to the next nodes
        if not self.pf_nd:
            fail_func = None
        elif type(self.pf_nd) == str:
            fail_func  = 'String reference: "%s"' % self.pf_nd
        else:
            fail_func = self.pf_nd.__dict__.get('function', None)
        ans =  '\nObject Node with name "%s" \n{\n function="%s",\n args="%s",\n kwargs"%s",\n success_nd.function="%s",\n fail_nd.function="%s",\n}\n' % (self.name, self.function, self.p_args, self.p_kwargs, success_func, fail_func)

        return ans

